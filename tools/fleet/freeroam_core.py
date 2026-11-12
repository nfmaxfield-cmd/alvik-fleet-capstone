#!/usr/bin/env python3
"""Home free roam, the controller (2026-11-05, Nick: "continuous simultaneous movement of multiple robots is the
accomplishment, that's a trophy"; 11:23: "the trophy is shiniest if they never stop moving").

No sockets and no clock of its own: every input comes in through on_*() with the time, every command goes out through
`out(name, text)`, every log row through `log(table, row)`. tools/fleet/freeroam.py runs it on the Pi (robots and
camera) and against tools/sim/fleet_sim.py; tools/fleet/freeroam_fastsim.py runs it many times faster than real time
for the stress tests. Same code in all three.

Layers, from planning to the last safety net:
  1. MENU      each robot's next move = turn x length x speed from a balanced menu: cells drawn least-used first
               (seeded; FREE_SHARE of the picks in a random order), the first one that is feasible is taken.
  2. PLAN      feasible = the end spot keeps EDGE_CC from every edge (plus its uncertainty) and the move, as a
               reservation in space AND time (turn spot, swept line sampled every SAMPLE_S, then standing at the end
               until its next move), never comes within PLAN_CC + execution margins of another robot's reservation
               in overlapping time windows (+ time slack for latency and timing error).
  3. FALLBACKS no menu move fits: off-menu (any heading, shorter), escape to open space, a holding turn on the spot
               (always possible: it never leaves its own circle). Deadlock (two robots holding next to each other):
               the lower-priority one yields (escape with the gap relaxed to REFLEX_CC). No waiting rung: Nick 11:23.
  4. REFLEX    every camera frame, ACTUAL positions and commanded motion, the next REFLEX_H_S: a predicted gap under
               REFLEX_CC -> the lower-priority mover changes speed (fspd), else steers (fsteer), else the other mover
               does; a hard stop only if nothing keeps it above HARD_CC within HARD_T_S (a "trophy break", logged).
  5. ROBOT     the robots' own front stop (follow_look_cm), edge guard, virtual border, heartbeat watchdog
               (firmware 2026-11-05b follow_wd_ms): their own safety stops, under all of the above.
Lost by the camera: dead reckoning with a growing bubble (BLIND_*), find-me wiggle when it grows too big; the old
cautious hold is kept behind lost_mode="hold".

(11-05d) MOTION STYLE (Nick, 5 Nov 16:06: "eliminate the stop and turn ... curving turns where velocity never drops
to 0"; "both options should always be available"): motion="curves" (the default here) or "stopturn" (everything
above, unchanged). In curves each robot on firmware 2026-11-05c or newer drives PIECES of constant curvature (follow
step R) that run into each other without slowing: the menu is curvature x length x speed; the next piece is planned
and queued while the current one runs; every piece ends where the robot can always LOOP AWAY (a circle of LOOP_R
inside the board, reserved for it), so when nothing else fits it circles in its own reserved disc instead of stopping;
the reflex slows to no less than V_MIN_AVOID or bends the curve (farc) instead of stopping; the robot's own front
check dodges (a tight curve away) instead of stopping. A robot on older firmware keeps stop-turn-drive (said at the
start, counted apart). Every moment a robot's speed reaches zero is logged with why (zeros.csv): the trophy is "never
stops".
"""
import math
import os
import random
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import safety_rules as SR   # noqa: E402  (e08 Phase A: the rules shared with run_plan and observe)
import fused_pose as FP     # noqa: E402  (e09 Phase B: one filter per robot, fed by the tracking layer and RAW)

VERSION = "freeroam-2026-11-12e17"   # e17 (home 11 Nov, the 08:26 and 08:58 runs): (1d) a camera position the tracking layer named by elimination or a wiggle is a CORRECTION: its pose set from the camera at once (fpose, never rejected by its filter), said "CORRECTED", never "moved in" / REJOINED, and a held robot rejoins on it (MIRLT_CAMERA_CORRECT=0: as e16); (1e) a re-seed is projected to now and never lands while the robot turns (the Pi's copy of its heading is ~0.3 s old: 13-15 deg median jump when it landed mid-turn, MIRLT_RESEED_TURN=0: as before); (2) no driving blind on late fixes: each robot's measured fix age and drift against the room its path has (MIRLT_LATE_FIX=0); after an off-plan heading re-set the next move waits for a short crawl the camera confirms (MIRLT_HEADING_CONFIRM=0); (6) each robot's drift per cm measured from its own corrections. Before: "freeroam-2026-11-11e16"
VERSION_E16 = "freeroam-2026-11-11e16"   # e16 (home free roam 11 Nov 08:45-08:48, commit 4732a48: Right stopped at an edge and stood; then "REJOINED ... moved in" 1 s after NEEDS A HAND with nobody there, and Center pushed it off): (7) an edge stop reverses along the robot's own recorded track as far as the derived edge rule needs (observe.track_back_target), checked against the others' reservations, then re-plans; it holds and asks only when the track is unknown or blocked (MIRLT_EDGE_BACKOUT=0: as before); (8a) "moved in" needs evidence: a lift or a sustained camera move past its own measured noise, or its own drive out (MIRLT_REJOIN_EVIDENCE=0: as before); (8b) a robot held or stuck at an edge, or rejoined and not yet driven away, is a fixed obstacle with its uncertainty (own pose against the camera) in every plan and the reflex (MIRLT_PINNED_OBSTACLE=0); (8c) contact: a standing robot the camera sees move past its noise, or whose accelerometer jolts, stops the robots near it; a driving robot whose camera progress falls far below its wheels' stops and backs off; contact.csv (MIRLT_CONTACT_WATCH=0); (8d) a question nobody answers (ASK_WAIT_S) makes the run decide by itself, as an unattended run does (MIRLT_ASK_UNATTENDED=1: keeps asking); (9b) a standing robot's doubt no longer grows with its fix's age (home: "cannot tell whether it is clear of the edge" 22 cm in; MIRLT_STAND_UNC=0). Before: "freeroam-2026-11-10e13b"   # e13b (10 Nov, Nick 13:34): a standing robot (its own odometry shows no motion since its last fix) keeps that fix: never lost while it stands, sparse reads confirm it (MIRLT_STAND_TRUST=0: before); the lost threshold grows only for a camera slower than about 2.5 pictures a second (1.5 x the longest recent gap). e13 (10 Nov): never give up blind (BLIND_RULE edge: a lost robot drives on while its bubble + body + stop cannot reach an edge; BUBBLE_CAP behind MIRLT_BLIND_RULE=cap); stray return (a robot outside the readable area comes back along ground it drove); never give up (a robot held at an edge waits, says it needs a hand and rejoins: EDGE_PARK off); risk-based no-pictures limit per robot
VERSION_E12 = "freeroam-2026-11-10a"   # 11-10a (e12): the curve-fit maths moved into module functions (fit_arc_points, arc_summary, arc_agree) so the calibration's curve phase uses the very same (free roam's own fitting and saving unchanged); a robot held at an edge for EDGE_PARK_S is parked and said loudly, the others roam on (MIRLT_EDGE_PARK=0: before)
VERSION_E = "freeroam-2026-11-07e"   # 11-07e (Nick 7 Nov): a robot put down at or over an edge first tries the e08 tilt recovery, a person asked only if that fails (MIRLT_START_EDGE_MODE=wait: always wait)
VERSION_D = "freeroam-2026-11-07d"   # 11-07d (e10, the 7 Nov run 2 incident): the queue watch (every tick, off plan or a queued piece breaking the edge rule from where the robot really is: re-aimed or stopped, heading re-set), the surface-mark verdict less the uncertainty, the start check (held at an edge)
VERSION_C = "freeroam-2026-11-07c"   # 11-07c (e10 Phase C): proven workspace, probe ahead, corner shuffle, empty moves end, pictures 1/min
VERSION_B = "freeroam-2026-11-07b"     # 11-07b (e09 Phase B): the filter's pose and uncertainty for the planner and the
                                     # edge rule (pose_source "ekf"; "blend" the one-step blend as before), fixes with a
                                     # source and sigma from the tracking layer, the identity wiggle, camera objects and
                                     # parked-robot moves as evidence, a picture asked when unsure near an edge alone,
                                     # B3 heading from the raw gyro (switch, off)
                                     # 11-07a (e08 Phase A): fused position (A1), the derived edge rule on every swept
                                     # path with a sure / unsure judgement (A2), tilt recovery tries (A3), heading watch,
                                     # stop lead, surface marks (A5), fading keep-outs (A6), held robots rejoin (A7)
                                     # 11-05d: continuous curves (motion style "curves", firmware 2026-11-05c)

# ------------------------------------------------------------------------------------------------- body and board
HALF = 6.72             # the 9.5 cm square's half diagonal: a circle round the body however it turns. Not chosen
BODY = 9.5              # the body's side, cm. Not chosen
# ------------------------------------------------------------------------------------------------- chosen numbers
EDGE_CC = 16.0          # chosen: a move's end spot (its middle) at least this far from every edge: the body's corner
                        # 9.3 cm in (roam.py's 3 in + 1.5 cm was 15.8). The robot's own border (6 cm) and edge guard
                        # are under it. DOE: yes, 14 / 16 / 20 cm against edge-guard trips and menu coverage (long
                        # moves need room); docs/doe-backlog.md
TURN_EDGE_CC = HALF + 2.5   # a robot turns on the spot only with its middle this far in (corners swing 6.72 cm).
                            # Follows from the body plus 2.5 cm chosen; no DOE
PLAN_GAP = 4.0          # chosen: planned reservations keep the circles round the bodies this far apart (squares face
                        # to face: 7.9 cm). DOE: yes, 3 / 4 / 6 cm against fallbacks per hour and reflex events
PLAN_CC = 2 * HALF + PLAN_GAP          # 17.4 cm middle to middle, before margins
REFLEX_GAP = 2.5        # chosen: the reflex acts when the predicted circles come closer than this. DOE: yes, with
REFLEX_CC = 2 * HALF + REFLEX_GAP      # PLAN_GAP (16.0 cm middle to middle)
HARD_CC = 2 * HALF + 0.5               # 13.9 cm: under this the circles touch (two squares can first touch at
                                       # 13.44 when corner meets corner). Hard stop only if heading for this
HARD_T_S = 1.2          # chosen: ... within this many seconds. No DOE: a reaction time (stop takes ~0.3 s at 13 cm/s)
REFLEX_H_S = 2.0        # chosen: reflex looks this far ahead (Nick: "~1-2 s"). DOE: 1.5 / 2 / 3 s against events
REFLEX_STEP_S = 0.1
CLEAR_HOLD_S = 1.0      # an adjustment is undone after the nominal prediction is clear (+2 cm) for this long
SPEED_OPTIONS = (0.6, 1.35, 0.35, 1.5, 0.15)   # chosen order: smallest change first, slower before faster
STEER_OPTIONS = (25, -25, 50, -50, 80, -80)    # deg off the current heading; chosen
V_MAX = 12.0            # the wheels' top speed in a straight line is ~12.5 cm/s (70 rpm, 1.7 cm wheels); a faster
                        # setting is not asked for. Not chosen
PLAN_M0 = 1.0           # chosen: execution margin at the start of a move (the camera's ~1 sigma, FIX_SIGMA_CM)
PLAN_M_PER_CM = 0.06    # ... growing this much per cm along the move (turn error 3 deg = 0.05 cm per cm, distance
PLAN_M_MAX = 3.0        # error 1-3 %), capped here: with camera fixes the robots stay within 2-3 cm of the line 95 %
                        # of the time (28 Oct, Phase 0 gate). DOE: yes, against the reflex events per hour
SLACK0_S = 0.6          # time slack of every reservation: camera frame 0.08 s + tracking 0.1 + command and ack
                        # latency with one resend 0.3 + step start 0.1. Chosen from those; DOE: no (a sum)
SLACK_FRAC = 0.15       # ... plus this share of the time since the move was committed (turn time and the speed ramps
                        # are estimates). Chosen; DOE: from moves.csv est_s vs dur_s after the first run
SAMPLE_S = 0.25         # reservation samples (3.3 cm apart at 13 cm/s; PLAN_M0 covers half of that). Not chosen
TURN_DPS = 75.0         # turn time = TURN_S0 + |deg| / TURN_DPS (40 rpm wheels: 92 deg/s, slower near the end).
TURN_S0 = 0.6           # Estimates; moves.csv logs the real times
ACCEL = 20.0            # firmware drive_accel, cm/s per s. Not chosen
# the balanced menu (Nick approved 11:23: "mostly balanced, a small share of free picks")
TURN_BINS = (-150, -90, -45, 0, 45, 90, 150)   # deg (+ = left), each drawn +-15 (0: +-10). Chosen; DOE note below
LEN_BINS = ((10, 20), (20, 35), (35, 55), (55, 80))   # cm. Chosen
SPEED_BINS = (5.0, 7.5, 10.0, 13.0)           # cm/s: the DOE loop's speeds (Nick's figures)
FREE_SHARE = 0.10       # chosen: 1 pick in 10 tries the cells in a random order, not least-used first, so the three
                        # robots do not chase the same rare long cells at once (they cluster on the long diagonals).
                        # DOE: yes, 0 / 10 / 25 % against coverage evenness and fallbacks per hour
PAUSE_S = 0.15          # between moves (the robot's own step queue; no visible stop). Not chosen
EMPTY_MOVE_PAUSE_S = 0.5    # (e10) CHOSEN: after a move with nothing to send, pick again this soon (DOE: no)
# dead reckoning (lost by the camera)
LOST_AFTER_S = 0.6      # no camera fix this long = lost (run_plan.BLIND_AFTER_S). Not chosen here
STAND_TRUST = os.environ.get("MIRLT_STAND_TRUST", "1") != "0"   # (e13, Nick 13:34) a standing robot keeps its fix
STAND_UNC = os.environ.get("MIRLT_STAND_UNC", "1") != "0"     # (e16, 9b) ... and its doubt does not grow while it
                        # stands (0: the fix's age grows the doubt at 10 cm/s as before e16)
STAND_CM = 1.0          # ... "not moved" = its own odometry moved under this since the fix: the camera's own noise (~1 cm),
STAND_DEG = 2.0         # and under 2 deg turned (the gyro's drift over a minute is far less). Not tuning numbers: below
                        # what a camera fix could tell apart anyway
LOST_GAPS = 1.5         # (e13) ... and never under 1.5 x the robot's longest recent gap between fixes (lost_after).
                        # CHOSEN (4 x the 90th percentile held a trapped robot 66 s in the 1 s-late-camera stress test);
                        # DOE row (the dark-room rate, e13)
BLIND_BASE = 2.0        # bubble p95 = 2.0 cm + 0.07 cm per cm driven blind: MEASURED, 29 Oct logs, 22,000 stretches,
BLIND_PER_CM = 0.07     # straight drives up to 40 cm (run_plan.BLIND_ERR_*). Beyond 40 cm extrapolated
BLIND_TURN_FRAC = 0.03  # heading error 3 % of the degrees turned blind (the e03 rerun: 0.85-0.90 deg per turn after the
                        # fit, 6 deg on a 173 deg turn before it): CHOSEN between the two, its lateral effect added
BUBBLE_CAP = 8.0        # BEFORE e13 (now MIRLT_BLIND_RULE=cap): over this the robot eased to a stop and wiggled
BLIND_RULE = os.environ.get("MIRLT_BLIND_RULE", "edge")   # (e13, Nick 12:51 "never give up", CLAUDE.md) "edge": a lost
                        # robot keeps moving on odometry and gyro while its bubble (grown per cm and degree blind) + half
                        # its body + its stop cannot reach an edge (blind_stop); the plans carry the bubble in their edge
                        # margins along the whole path. No fixed bubble cap. "cap": BUBBLE_CAP as before
REACQUIRE_CM = 2.5      # seen again: its pose is set to the camera's when they differ by more (run_plan). Not chosen
REACQUIRE_MAX_CM = 12.0  # ... over this, 3 agreeing fixes first (the camera may have the wrong robot)
WIGGLE_DEG = 20.0       # find-me wiggle: +-20 deg on the spot. Chosen; no DOE (any visible motion works)
LOST_HOLD_S = 1.5       # lost_mode "hold" (the old roam.py rule): halt a mover unseen this long
# robots
STUCK_S = 3.0           # driving with no camera travel (< 1 cm) for this long = stuck. Chosen; DOE no
SILENT_S = 6.0          # nothing heard for this long: "silent", kept as an obstacle where the camera sees it
OUT_SILENT_S = 120.0    # silent this long: out for the rest of the run (before e08; now behind silent_out=True: A7 holds
                        # it as an obstacle and rejoins it when the camera and its own pose agree again)
RESEND_S = 0.3          # a step or control not acknowledged is sent again after this (run_plan --resend-s 0.3)
HEARTBEAT_S = 0.5       # something goes to every robot at least this often (its watchdog: follow_wd_ms 1500)
WD_MS = 1500            # the robots' watchdog. Chosen: 15 camera fixes; DOE no (a safety timeout)
STATUS_S = 20.0         # battery asked this often
try:                    # (e08, A11) one source of truth: engine_checks.battery_floor("freeroam") = safety_rules' 10 %
    import engine_checks as _EC
    BATTERY_FLOOR = _EC.battery_floor(run_type="freeroam")     # Nick: each robot drives down to 10 %
except Exception:  # noqa: BLE001
    BATTERY_FLOOR = 10
PARK_EDGE_CC = EDGE_CC + 3.0   # a parked robot's spot: near an edge, out of the way
FRESH_FW = "2026-11-05b"       # fspd / fsteer / follow_wd_ms
# (11-05c, the first real run, 5 Nov 14:35) camera timing, discovered keep-outs, backing off, the edge trap
CAM_DELAY0 = 0.15       # a fix's delay (picture taken -> here) assumed until measured. The real home Pi: 0.42-0.46 s
                        # (the camera bridge's own processing, bridge/events.jsonl, 5 Nov). Measured online after ~10 fixes
CLOCK_DEADBAND_S = 0.25 # the camera's and this controller's clocks are taken as one clock unless the fixes line up with
                        # the robots' own poses this far or more apart (the own poses' WiFi lag is 0.1-0.2 s: 5 Nov logs).
                        # Chosen; no DOE (a threshold above a measured lag)
CLOCK_EVERY_S = 5.0     # the clock alignment is redone this often (only from robots that move)
VERIFY_CM = 5.0         # a camera spot far from the robot's own pose (> REACQUIRE_MAX_CM) is believed only after the
                        # camera has followed the robot's own motion for this far with an unchanged offset (5 Nov: the
                        # camera put Center 45 cm off, on Right's spot, and the controller set Center's pose there).
                        # Chosen; DOE: no (any travel the camera must agree with; 5 cm is 5 camera sigmas)
VERIFY_STILL_S = 2.0    # ... or, standing still, the camera's spot steady this long (and nobody else there). Chosen
WALL_MIN_IN = 3.0       # ... and at least this far (closer is the calibration's own error and the felt point's). Chosen
WALL_MAX_IN = 20.0      # a side moves in at most this far from the calibration's (more: an object). Chosen
KEEP_R = 2.0            # a felt obstacle point (front stop, edge guard) is a keep-out disc this big: the front sensor's
                        # zones are ~1 cm apart at 3 cm. Chosen; DOE no
KEEP_GAP = 1.0          # planned bodies' circles keep this much more from a keep-out. Chosen; DOE with EDGE_CC
BACK_CM = (8.0, 6.0, 4.0)   # backing off after the robot's own front stop: the longest of these that fits. Chosen:
                            # 8 cm takes a 2.9 cm obstacle past the turn check (safe_front_cm 7) with room. DOE no
BACK_V = 5.0            # cm/s: backwards is blind (no rear sensor): slowest menu speed
CORNER_BACK_CM = (14.0, 10.0, 6.0, 3.0, 2.0, 1.5)   # (e10, C4) the edge trap's reverses: + 2 and 1.5 cm. Near a CORNER,
                            # facing along neither side, the best inward reverse is short (the e09 hung-robot case with
                            # the filter in charge: freed 7.1 cm from a side, a 2 cm reverse gains 0.9 cm and reaches the
                            # 7.7 cm a tight edge turn needs; 3 cm or more gains nothing)
RECOVER_EXTRA_CM = 6.0      # (e10) CHOSEN: a recovery reverse may go this much past its usual length for room; DOE: no
PROBE_V = 3.0               # (e10) cm/s: a probe's speed (feel_table's crawl, the warm-up's side-feel speed): not chosen anew
PROBE_IN_CM = 8.0           # (e10) CHOSEN: a probe's planned end lies this far past the last proven ground; the bottom
                            # zone sees a drop 9-11 cm ahead of the wheels (feel_table), so it stops before reaching it
PROBE_MAX_L = 50.0          # (e10) CHOSEN: the longest probe drive (cm)
PROBE_REACH_CM = 60.0       # (e10) CHOSEN: frontiers looked for within this of the robot
PROBE_EVERY_S = 8.0         # (e10) CHOSEN: a robot probes again at most this often; DOE: yes, against area proven per
                            # minute and time lost to probing
FELT_HALF_W = 4.0           # (e10) the strip a robot's wheels prove (its track 8.9 cm: HALF_TRACK, less 0.45): not chosen
BOTTOM_REACH_CM = 9.0       # (e10) the bottom zone notices a drop 9-11 cm ahead of the wheels (feel_table): the low end
LINE_REACH_CM = 4.0         # (e10) the line sensors sit about 3.5-4 cm ahead of the middle (feel_table)
PICTURE_EVERY_S = 60.0      # (e10) Claude's default (Nick may change): one "unsure near an edge" picture per robot a minute
SHUFFLE_MARGIN_CM = 1.0     # (e10) CHOSEN: the corner shuffle keeps each body corner this + its uncertainty inside; DOE: yes,
                            # against hangs per hour in corners (the board's edge bench test)
INWARD_GAIN_CM = 0.5        # (e10) CHOSEN: such a reverse must end at least this much further in (was 1.0); DOE: no
EDGE_TURN_MIN = HALF + 1.0  # the edge trap's last resort: a turn on the spot with the middle this far in (corners swing
                            # 6.72 cm: they stay 1 cm inside the edge). Only after reversing found nothing


# (11-05d) continuous curves
ARC_FW = "2026-11-05c"  # follow step R, farc, coast, dodge
V_MIN_AVOID = 5.0       # cm/s: the slowest an avoiding curve robot is made to go (never 0). EVIDENCE-BASED: 2 / 3 / 5
                        # cm/s compared in the stress simulator, 4 seeds x 1 h each (docs/build-reports/e05-nov05-curves.md,
                        # "Minimum speed"): with a dead robot in the way 2 cm/s touched 3 times, 3 cm/s came to 0.7 cm,
                        # 5 cm/s to 1.6 cm with the fewest speed-zero moments; the other seeds alike. A finer DOE (4 / 5 /
                        # 6 / 7 cm/s, more seeds) would pin it down
K_BINS = (-1 / 12.0, -1 / 25.0, -1 / 50.0, 0.0, 1 / 50.0, 1 / 25.0, 1 / 12.0)   # curvature 1/cm (+ = left): radius
                        # 12, 25, 50 cm and straight. Chosen (12 cm: the tightest the wheels do at 9 cm/s); DOE: yes,
                        # with LOOP_R, against fallbacks per hour and coverage
ARC_LEN_BINS = ((10, 20), (20, 35), (35, 55))   # cm along the curve. Chosen (shorter than stop-turn's: the next piece
                        # is planned while this one runs, so a piece is a commitment of ~2-8 s)
ARC_SPEED_BINS = (5.0, 7.5, 10.0, 12.0)        # cm/s (the DOE loop's, top capped at what the wheels do)
WHEEL_TOP = 12.3        # cm/s: the outer wheel's top speed on a curve (70 rpm, 1.7 cm wheels: 12.5). Not chosen
HALF_TRACK = 4.45       # cm (setDrive's 89 mm track). Not chosen
LOOP_R = 10.0           # the loop-away circle's radius, cm: every piece ends where a circle this big (on one side)
                        # stays inside the board and clear of everyone's reservations. Chosen: the outer wheel at 2.3x
                        # the inner, 6.3 cm/s at the edge of the disc; DOE: yes, 8 / 10 / 14 cm against fallbacks
LOOP_EDGE = HALF + 2.5  # a curve's samples keep their middle this far in (the corners 2.5 cm inside). Chosen = TURN_EDGE_CC
DODGE_K = 1 / 6.0       # firmware arc_dodge_r 6 cm. Not chosen here
COAST_S = 0.8           # firmware arc_coast_ms. Not chosen here
AVOID_K = (1 / 30.0, -1 / 30.0, 1 / 15.0, -1 / 15.0, 1 / 10.0, -1 / 10.0)   # the reflex's bends (farc), smallest first.
                        # Chosen; DOE with REFLEX_GAP
ARC_FIT_MIN = 15        # (11-05d) the curves' calibration, discovered from the run: at least this many fitted curve pieces
                        # (|k| >= 1/40, >= 35 deg) before arc_k_scale / arc_d_scale are saved on a parked robot. Chosen
ARC_FIT_CI = 0.03       # ... and the median's 95 % half-width under this. Chosen
EDGE_AWAY = True        # (11-05d) a curve from a stand near an edge may run along it if it never gets closer (switch)
REAIM = True            # (11-05d) an overdue curve piece is re-aimed with farc (switch)
# (e10, the 7 Nov run 2 incident: Right's 55 cm straight ran 25 deg off, the piece queued behind it was planned from
# the straight's PLANNED end, ran from where the robot really was, and its line-sensor stop 9 cm from the edge by the
# camera was called a surface mark; Right ended hung over the y = 0 edge) switches, decided automatically (on), the old
# way behind each (CLAUDE.md "keep features, switch them off"):
QUEUE_WATCH = os.environ.get("MIRLT_QUEUE_WATCH", "1") != "0"   # (a, c) every tick the running piece is checked
                        # against the fused position (and the camera's own fix) and its queued pieces against the edge
                        # rule from where the robot really is and the way it really goes; off plan: re-aimed or stopped
MARK_UNC = os.environ.get("MIRLT_MARK_UNC", "1") != "0"         # (b) the surface-mark verdict uses the camera's
                        # edge distance minus the fused pose's uncertainty (near an edge a line trip is an edge)
START_EDGE = os.environ.get("MIRLT_START_EDGE", "1") != "0"     # (d) a robot that starts over or at an edge: checked
START_EDGE_MODE = os.environ.get("MIRLT_START_EDGE_MODE", "recover")   # (d; Nick 7 Nov) "recover": it first tries to
                        # free itself the e08 tilt-recovery way (straight back when the camera and its own pose agree
                        # that backing is inward, then the other tries) and a person is asked only if that fails;
                        # "wait": it always waits for a person (the first e10 way, the switch)
OFFPLAN_CM = 4.0        # (a) off plan: the fused position (or the camera's fix) this far ACROSS the running piece's path
                        # (SURE_TOL_CM + PLAN_M0: own pose and camera agree within 3 cm, the plan's 1 cm margin).
                        # 25 deg off is caught after ~10 cm. Chosen; DOE with OFFPLAN_S
OFFPLAN_S = 0.3         # ... for this long (one camera fix's noise is not enough). Chosen
COURSE_S = 1.5          # the way it really goes: a straight line through the camera's fixes over this long
COURSE_MIN_CM = 3.0     # ... when they spread at least this far (else the filter's / its own heading)
COURSE_ERR_DEG = 10.0   # its own heading this far off the camera's course (for COURSE_ERR_S): the rest of the piece and
COURSE_ERR_S = 0.5      # the queued ones are checked from where it really is, the way it really goes. Chosen; DOE
COURSE_RESET_DEG = 15.0 # its own heading this far from the camera's course while off plan: re-set (fpose), stop, re-plan
HEADING_CLICKS = os.environ.get("MIRLT_HEADING_CLICKS", "1") != "0"   # (e13, Nick 13:01) after a click, a straight drive
                        # and a second click give the heading; 0 = the gyro's heading kept
NOPIC_RULE = os.environ.get("MIRLT_NOPIC_RULE", "risk") != "off"   # (e13) no pictures: each robot's risk-based limit
                        # (slowed, then held); "off": as e12 (the lost-robot rules alone after the bridge's message)
STRAY_RETURN = os.environ.get("MIRLT_STRAY_RETURN", "1") != "0"   # (e13) a robot outside the readable area comes back
                        # (only where the workspace is limited to it: school free roam). 0 = before e13 (lost there)
EDGE_PARK = os.environ.get("MIRLT_EDGE_PARK", "0") == "1"   # (e13, Nick 10 Nov "robots never give up"): OFF by default.
                        # A robot held at an edge now stays in the run: still and safe, it says loudly that it needs a
                        # hand (NEEDS A HAND: event, state, Mission Control) and keeps checking; moved in, it rejoins by
                        # itself; held, its own battery floor still parks it (so the run can end). MIRLT_EDGE_PARK=1:
                        # e12's park after EDGE_PARK_S, kept as the switch.
                        # (e12, Nick 10 Nov) a robot held at an edge (put down there
                        # and not moved in; tilted with no safe way out; unsure near an edge and held) for EDGE_PARK_S is
                        # PARKED for the rest of the run: it stays still where it is (kept out by the others), it counts
                        # as out (so the run is never held up waiting for it: home ends when the others park), and it is
                        # said loudly at once, in the state, at the end and on Mission Control. 0 = before e12 (held,
                        # retried and asked for the whole run)
# (e16, home 11 Nov 08:45-08:48, run 20261111-082651-freeroam) switches, decided automatically (on), the old way behind
# each (CLAUDE.md "keep features, switch them off"):
EDGE_BACKOUT = os.environ.get("MIRLT_EDGE_BACKOUT", "1") != "0"        # (7) an edge stop reverses along its own track
REJOIN_EVIDENCE = os.environ.get("MIRLT_REJOIN_EVIDENCE", "1") != "0"  # (8a) "moved in" only on evidence
PINNED_OBSTACLE = os.environ.get("MIRLT_PINNED_OBSTACLE", "1") != "0"  # (8b) held at an edge: a fixed obstacle + doubt
CONTACT_WATCH = os.environ.get("MIRLT_CONTACT_WATCH", "1") != "0"      # (8c) pushes and collisions noticed
ASK_UNATTENDED = os.environ.get("MIRLT_ASK_UNATTENDED", "0") == "1"    # (8d) 1 = keep asking when nobody answers
CAMERA_CORRECT = os.environ.get("MIRLT_CAMERA_CORRECT", "1") != "0"   # (e17, 1d) the layer's elimination = a correction
RESEED_TURN = os.environ.get("MIRLT_RESEED_TURN", "1") != "0"         # (e17, 1e) no re-seed lands mid-turn; projected
LATE_FIX = os.environ.get("MIRLT_LATE_FIX", "1") != "0"               # (e17, 2) slow or wait on late fixes
HEADING_CONFIRM = os.environ.get("MIRLT_HEADING_CONFIRM", "1") != "0"   # (e17, 2) camera-confirmed heading after a re-set
NEVER_STILL = os.environ.get("MIRLT_NEVER_STILL", "1") != "0"     # (e17, Nick 12 Nov standing rule) held / unsure /
                        # needs-a-hand / lost-at-the-limit robots keep creeping toward safety and being found; 0: e16's
                        # hold-and-wait
CREEP_MAX_CM = 8.0      # (e17) CHOSEN: a creep step's longest (a robot's own 9.5 cm body or so) ...
CREEP_MIN_CM = 2.0      # ... and shortest (over the camera's noise, so the camera sees it go)
CREEP_UNC0_CM = 5.0     # CHOSEN: step and speed scale as UNC0 / (UNC0 + its uncertainty): 5 cm doubt halves them.
                        # DOE: yes, with CREEP_MAX_CM (time to be found again vs falls per hour)
CREEP_V_MIN = 2.0       # cm/s: the firmware's slowest drive (driveLine's 2 cm/s floor). Not chosen
RESEED_TURN_DPS = 8.0   # (e17) a re-seed waits while its own heading turns faster than this: at the measured ~0.3 s
                        # landing lag that is under 3 deg planted (the camera heading's own sigma). Derived; DOE: no
RESEED_WAIT_MAX_S = 3.0  # (e17) ... at most this long (a robot circling for ever is re-seeded anyway, said). Chosen
LATE_SLOW_MIN = 0.2     # (e17, 2) CHOSEN: under 0.2 of its speed it waits for a fix instead (fspd 0). DOE: yes, with
                        # the drift record's percentile (stops per hour vs closest approach to an edge)
LATE_REC_MIN = 3        # (e17) corrections needed before a robot's own drift rate replaces the measured prior
                        # (BLIND_PER_CM 0.07 cm per cm, 29 Oct). Chosen
HD_CONFIRM_DEG = 5.0    # (e17) CHOSEN: the heading the crawl checks is good to this (half COURSE_ERR_DEG). DOE: yes
HD_CONFIRM_MAX_CM = 12.0   # (e17) CHOSEN: the crawl's longest (it is derived from the camera's noise, up to this)
MOVED_IN_K = 4.0        # (8a) sigmas of the robot's own camera noise a move must exceed (one-sided, ~3e-5 by chance per
                        # check on Gaussian noise). Statistical, not a tuning; DOE: no
CONTACT_K = 4.0         # (8c) the same test for a standing robot pushed (its camera spot moves past its own noise)
IMPACT_K = 8.0          # (8c) an accelerometer jolt this many robust sigmas of its own standing noise. CHOSEN (the
IMPACT_FLOOR_G = 0.05   # floor: the 3' "felt it roll" jolt is 0.03 g); DOE: yes, on raw.csv of bumps and of stands
PROGRESS_MIN = 0.3      # (8c) a drive whose camera progress over PROGRESS_WIN_S is under this share of its wheels', on
PROGRESS_WIN_S = 1.0    # PROGRESS_N checks in a row (half a second apart) is pushing or blocked. CHOSEN: at 0.5 and one
PROGRESS_N = 2          # check the run-2 simulation stopped 2 free drives (camera 0.48-0.49 of the wheels while its
                        # fixes lag a starting piece); a robot pushing a held one reads ~0. DOE: yes (0.2 / 0.3 / 0.4 x
                        # 1-3 checks against false stops on the real logs and stops caught in the push simulation)
ASK_WAIT_S = 30.0       # (8d) a question unanswered this long: nobody is watching. CHOSEN (a person in the room answers
                        # in seconds); no DOE: a human's response time, not a physical quantity
NEED_HAND_SAY_S = 60.0  # (e13) the NEEDS A HAND line is said again this often while it lasts: display only, no DOE
EDGE_PARK_S = 60.0      # chosen: three of the recovery's 20 s retry cycles (RECOVER_RETRY_S) with no way out. DOE: yes, a
                        # small one (30 / 60 / 120 s against robots freed by a later retry vs minutes stood waiting), from
                        # the recovery.csv of real runs
FARC_MIN_S = 0.6        # a re-aim (farc) only with this much of the running piece left (else the queued piece may have
                        # started on the robot by the time the farc arrives: halted instead)
START_EDGE_CM = HALF + 1.0          # (d) the proof drive (its first move a turn on the spot) needs the middle this far in
START_CORNER_CM = 1.0               # (d) to start at all, every corner of the body this far inside the board by the camera
                                    # (its ~1 cm error): a robot over the edge waits; one just inside it starts, and e08's
                                    # edge-trap rule backs it out (Right 5 Nov, 6.6 cm in, facing out). Chosen; no DOE
START_CLEAR_CM = START_CORNER_CM + 2.0  # ... and its corners this far in once held (a person moved it in)
CURVE_SAMPLE_CM = 3.0   # reservation samples along a curve at most this far apart (PLAN_M0 covers half). Not chosen


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def arc_at(x, y, h, k, s):
    """Pose after s cm (scalar or array) along a curve of curvature k (1/cm, + = left) from (x, y) facing h deg."""
    s = np.asarray(s, float)
    hr = math.radians(h)
    if abs(k) < 1e-6:
        return x + s * math.cos(hr), y + s * math.sin(hr), h + 0.0 * s
    a = hr + k * s
    return x + (np.sin(a) - math.sin(hr)) / k, y - (np.cos(a) - math.cos(hr)) / k, h + np.degrees(k * s)


def loop_centre(x, y, h, side):
    """The centre of the loop-away circle of a robot at (x, y) facing h, looping left (side +1) or right (-1)."""
    hr = math.radians(h)
    return x - math.sin(hr) * side * LOOP_R, y + math.cos(hr) * side * LOOP_R


def v_cap(k):
    """The fastest a curve of curvature k can be driven (its outer wheel at WHEEL_TOP)."""
    return WHEEL_TOP / (1.0 + HALF_TRACK * abs(k))


def fit_arc_points(pts, k, L, min_k=1 / 40.0, min_sweep_deg=35.0, min_pts=12):
    """(11-10a, moved out of FreeRoam.arc_fit unchanged, so the calibration's curve phase uses the same maths) The
    camera's spots along one finished curve piece fitted with a circle (its last 80 %: the start bends from the last
    piece): (k ratio = fitted curvature / asked, length ratio = the camera's arc (angle swept x radius) / asked, fitted
    radius) or None when the piece is too straight, too short or the fit is not believable. min_k: free roam 1/40
    (radius 40 cm or tighter); the calibration passes its own (it drives radius 50 too)."""
    if abs(k) < min_k or abs(k * L) < math.radians(min_sweep_deg) or len(pts) < min_pts:
        return None
    P = np.array(pts[len(pts) // 5:], float)
    A = np.c_[2 * P[:, 0], 2 * P[:, 1], np.ones(len(P))]
    sol, *_ = np.linalg.lstsq(A, (P ** 2).sum(1), rcond=None)
    cx, cy = sol[0], sol[1]
    R = math.sqrt(max(1e-6, sol[2] + cx * cx + cy * cy))
    if not (2.0 < R < 200.0):
        return None
    Q = np.array(pts, float)
    ang = np.unwrap(np.arctan2(Q[:, 1] - cy, Q[:, 0] - cx))
    kr = (1.0 / R) / abs(k)
    dr = abs(ang[-1] - ang[0]) * R / L
    if not (0.6 < kr < 1.6 and 0.6 < dr < 1.6):
        return None
    return kr, dr, R


def arc_summary(fits):
    """(median k ratio, its 95 % half-width, median length ratio, its half-width, n) of [(k ratio, length ratio)], or
    None under 3 fits. (11-10a: FreeRoam.arc_result's maths, shared.)"""
    if len(fits) < 3:
        return None
    a = np.array(fits)
    n = len(a)
    hw = lambda v: 1.2533 * 1.96 * float(np.std(v)) / math.sqrt(n)     # noqa: E731 (median's standard error)
    return float(np.median(a[:, 0])), hw(a[:, 0]), float(np.median(a[:, 1])), hw(a[:, 1]), n


def arc_agree(res, cur=(1.0, 1.0), min_n=None, ci=None):
    """(11-10a: arc_save's agreement rule, shared) (ok, new arc_k_scale, new arc_d_scale): ok when there are min_n
    (ARC_FIT_MIN) fits, both medians' 95 % half-widths are under ci (ARC_FIT_CI) and the new values stay in 0.8-1.25."""
    kr, kh, dr, dh, n = res
    min_n = ARC_FIT_MIN if min_n is None else min_n
    ci = ARC_FIT_CI if ci is None else ci
    ks, ds = cur[0] / kr, cur[1] / dr
    ok = n >= min_n and kh <= ci and dh <= ci and 0.8 <= ks <= 1.25 and 0.8 <= ds <= 1.25
    return ok, ks, ds


def edge_dist(rect, x, y):
    """Distance from (x, y) to the board's nearest edge. (e10, C2) rect may be a proven_workspace.ProvenWorkspace: then
    the distance to the nearest ground that is not proven table (the switch: a plain rectangle, as before)."""
    f = getattr(rect, "edge_dist", None)
    if f is not None:
        return f(x, y)
    return min(x - rect[0], rect[2] - x, y - rect[1], rect[3] - y)


def edge_dist_v(rect, xs, ys):
    """edge_dist for numpy arrays of points."""
    f = getattr(rect, "dist_v", None)
    if f is not None:
        return f(xs, ys)
    return np.minimum(np.minimum(xs - rect[0], rect[2] - xs), np.minimum(ys - rect[1], rect[3] - ys))


def turn_s(deg):
    return 0.0 if abs(deg) < 3.0 else TURN_S0 + abs(deg) / TURN_DPS


def drive_s(L, v):
    return L / v + v / ACCEL + 0.3


# ------------------------------------------------------------------------------------------------ reservations
INF = 1e12


class Traj:
    """A robot's reservation: samples (t_lo, t_hi, x, y, margin, slack) and the open-ended stand at its end."""

    def __init__(self, rows):
        a = np.array(rows, dtype=float).reshape(-1, 6)
        self.lo, self.hi, self.x, self.y, self.m, self.sl = a.T

    @staticmethod
    def stand(t, x, y, m=PLAN_M0, slack=SLACK0_S):
        return Traj([(t, INF, x, y, m, slack)])

    @staticmethod
    def move(t0, p0, h0, turn, L, v, m0=PLAN_M0, bubble_grow=0.0, slack0=SLACK0_S):
        """Turn on the spot by `turn` deg at p0, then drive L cm at v cm/s. bubble_grow: extra margin per cm (lost)."""
        rows = []
        h1 = h0 + turn
        tt = turn_s(turn)
        if tt > 0:
            rows.append((t0, t0 + tt, p0[0], p0[1], m0, slack0 + SLACK_FRAC * tt))
        td = drive_s(L, v) if L > 0 else 0.0
        ux, uy = math.cos(math.radians(h1)), math.sin(math.radians(h1))
        n = max(1, int(math.ceil(td / SAMPLE_S))) if L > 0 else 0
        for k in range(n):
            a, b = t0 + tt + k * td / n, t0 + tt + (k + 1) * td / n
            s = min(L, L * (k + 0.5) / n)
            m = m0 + min(PLAN_M_MAX, PLAN_M_PER_CM * s) + bubble_grow * s
            rows.append((a, b, p0[0] + ux * s, p0[1] + uy * s, m, slack0 + SLACK_FRAC * (b - t0)))
        t1 = t0 + tt + td
        s = L
        m = m0 + min(PLAN_M_MAX, PLAN_M_PER_CM * s) + bubble_grow * s
        rows.append((t1, INF, p0[0] + ux * L, p0[1] + uy * L, m, slack0 + SLACK_FRAC * (t1 - t0)))
        tr = Traj(rows)
        tr.t_end, tr.end, tr.h_end = t1, (p0[0] + ux * L, p0[1] + uy * L), h1
        tr.v = v
        return tr


def curve_traj(t0, segs, side, m0=PLAN_M0, bubble_grow=0.0, slack0=SLACK0_S, tail=True):
    """(11-05d) A curve robot's reservation: segs = [(x, y, h, k, L, v)] driven one after the other from t0 (each at
    its own speed: the firmware's ramps between them are under the slack), sampled every SAMPLE_S or CURVE_SAMPLE_CM;
    then (tail) the loop-away disc at the last piece's end, looping to `side`, reserved for ever (its margin covers the
    circle: LOOP_R)."""
    rows = []
    t, s_tot = t0, 0.0
    end = None
    for (x, y, h, k, L, v) in segs:
        v = max(0.5, v)
        td = L / v
        n = max(1, int(math.ceil(max(td / SAMPLE_S, L / CURVE_SAMPLE_CM))))
        ss = (np.arange(n) + 0.5) * L / n
        xs, ys, _ = arc_at(x, y, h, k, ss)
        for j in range(n):
            a, b = t + j * td / n, t + (j + 1) * td / n
            st = s_tot + ss[j]
            m = m0 + min(PLAN_M_MAX, PLAN_M_PER_CM * st) + bubble_grow * st
            rows.append((a, b, float(xs[j]), float(ys[j]), m, slack0 + SLACK_FRAC * (b - t0)))
        t += td
        s_tot += L
        ex, ey, eh = arc_at(x, y, h, k, L)
        end = (float(ex), float(ey), float(eh))
    m = m0 + min(PLAN_M_MAX, PLAN_M_PER_CM * s_tot) + bubble_grow * s_tot
    if tail and end is not None:
        c = loop_centre(end[0], end[1], end[2], side)
        rows.append((t, INF, c[0], c[1], m + LOOP_R, slack0 + SLACK_FRAC * (t - t0)))
    elif end is not None:
        rows.append((t, INF, end[0], end[1], m, slack0 + SLACK_FRAC * (t - t0)))
    tr = Traj(rows)
    tr.t_end, tr.end, tr.h_end, tr.side = t, (end[0], end[1]), end[2], side
    tr.loop_c = loop_centre(end[0], end[1], end[2], side) if tail else None
    tr.v = segs[-1][5] if segs else None
    return tr


def concat(*ts):
    rows = np.vstack([np.stack([t.lo, t.hi, t.x, t.y, t.m, t.sl], 1) for t in ts])
    out = Traj(rows)
    out.end = getattr(ts[-1], "end", (rows[-1, 2], rows[-1, 3]))
    return out


def clearance_profile(A, B, now=None):
    """Per sample of A: the smallest (distance - margins) to B over overlapping time windows (+inf: none)."""
    lo_a, hi_a = A.lo[:, None] - A.sl[:, None] - B.sl[None, :], A.hi[:, None]
    over = (lo_a <= B.hi[None, :]) & (B.lo[None, :] - A.sl[:, None] - B.sl[None, :] <= hi_a)
    if now is not None:
        over &= (A.hi[:, None] >= now) & (B.hi[None, :] >= now)
    d = np.hypot(A.x[:, None] - B.x[None, :], A.y[:, None] - B.y[None, :]) - A.m[:, None] - B.m[None, :]
    d = np.where(over, d, np.inf)
    return d.min(axis=1)


def clearance(A, B, now=None):
    """The smallest (distance - margins) between two reservations over overlapping time windows (+ slack). +inf if
    they never overlap in time."""
    lo_a, hi_a = A.lo[:, None] - A.sl[:, None] - B.sl[None, :], A.hi[:, None]
    over = (lo_a <= B.hi[None, :]) & (B.lo[None, :] - A.sl[:, None] - B.sl[None, :] <= hi_a)
    if now is not None:
        over &= (A.hi[:, None] >= now) & (B.hi[None, :] >= now)
    if not over.any():
        return float("inf")
    d = np.hypot(A.x[:, None] - B.x[None, :], A.y[:, None] - B.y[None, :]) - A.m[:, None] - B.m[None, :]
    return float(d[over].min())


# ------------------------------------------------------------------------------------------------ the menu
class Menu:
    def __init__(self, rng):
        self.rng = rng
        self.cells = [(a, b, c) for a in range(len(TURN_BINS)) for b in range(len(LEN_BINS))
                      for c in range(len(SPEED_BINS))]
        self.n = {c: 0 for c in self.cells}
        self.n_robot = {}

    def order(self, name, free=False):
        cs = list(self.cells)
        self.rng.shuffle(cs)
        if free:
            return cs
        nr = self.n_robot.setdefault(name, {})
        return sorted(cs, key=lambda c: (self.n[c], nr.get(c, 0)))   # stable: the shuffle breaks ties

    def draw(self, cell):
        a, b, c = cell
        jit = 10.0 if TURN_BINS[a] == 0 else 15.0
        turn = TURN_BINS[a] + self.rng.uniform(-jit, jit)
        L = self.rng.uniform(*LEN_BINS[b])
        return turn, L, SPEED_BINS[c]

    def use(self, cell, name):
        self.n[cell] += 1
        nr = self.n_robot.setdefault(name, {})
        nr[cell] = nr.get(cell, 0) + 1

    @staticmethod
    def label(cell):
        a, b, c = cell
        return "t%+d_L%d-%d_v%g" % (TURN_BINS[a], LEN_BINS[b][0], LEN_BINS[b][1], SPEED_BINS[c])


class CurveMenu(Menu):
    """(11-05d) The curves' balanced menu: curvature x length x speed (7 x 3 x 4 = 84 cells), least-used first, seeded,
    the same free share. A curve's speed is capped by what its outer wheel can do (v_cap)."""

    def __init__(self, rng):
        self.rng = rng
        self.cells = [(a, b, c) for a in range(len(K_BINS)) for b in range(len(ARC_LEN_BINS))
                      for c in range(len(ARC_SPEED_BINS))]
        self.n = {c: 0 for c in self.cells}
        self.n_robot = {}

    def draw(self, cell):
        a, b, c = cell
        k = K_BINS[a] * self.rng.uniform(0.85, 1.15) if K_BINS[a] else self.rng.uniform(-1 / 300.0, 1 / 300.0)
        L = self.rng.uniform(*ARC_LEN_BINS[b])
        return k, L, min(ARC_SPEED_BINS[c], v_cap(k))

    @staticmethod
    def label(cell):
        a, b, c = cell
        k = K_BINS[a]
        return "%s_L%d-%d_v%g" % ("straight" if not k else "r%+d" % round(1 / k), ARC_LEN_BINS[b][0],
                                  ARC_LEN_BINS[b][1], ARC_SPEED_BINS[c])


# ------------------------------------------------------------------------------------------------ one robot
class Robot:
    def __init__(self, name, rank):
        self.name, self.rank = name, rank
        self.cam = None                 # (t, x, y, th or None)
        self.cam_hist = []              # [(t, x, y)] last ~4 s
        self.own = None                 # (t, x, y, th, state, cur)
        self.own_hist = []
        self.mode = "start"             # start, idle, move, parking, parked, out
        self.out_why = None
        self.move = None                # dict: kind, cell, steps, traj, t0, v, end, h, phase
        self.traj = None
        self.next_id = 1
        self.steps = {}                 # sid -> dict(text, status, t_sent)
        self.controls = []              # dict(text, t_last, kind)
        self.scale = 1.0
        self.adj = None                 # reflex adjustment in force (event dict)
        self.battery = None
        self.fw = None
        self.heard = None
        self.silent = False
        self.silent_t = None
        self.bat_logged = (None, -1e9)
        self.lost = None                # dict while lost: t0, drive, turn, bubble, last_own
        self.bubble = 0.0
        self.hold_n = 0                 # holding turns in a row
        self.hold_sign = 1
        self.busy_until = 0.0
        self.last_sent = 0.0
        self.last_status = -1e9
        self.wiggle_due = 0.0
        self.reacq = []
        self.n_moves = 0
        self.stopped_hard = None        # t of a hard stop (trophy break) in force
        self.lost_hold = False
        self.cam_arr = None             # when the newest fix arrived (this controller's clock)
        self.cam_raw = []               # [(t as stamped, x, y)] for the clock alignment
        self.cam_off = None             # camera minus own pose at the picture's time, the last accepted fix
        self.back_off = None            # dict after its own front stop: the next move backs away
        self.pc = None                  # (t, pos) cache
        self.disagree_t = -1e9
        self.jump_n = 0
        self.curve = False              # (11-05d) drives continuous curves (motion "curves" and firmware 2026-11-05c+)
        self.queue = []                 # curve pieces sent after the current one
        self.coast = None               # dict(t, k, v, dodge): the robot rolls on round its last curve (no piece)
        self.zero_why = None            # (why, t): the latest reason this robot's speed may reach zero
        self.zero = None                # dict while its speed is zero (motion rows): t, why
        self.zeros = []                 # closed speed-zero moments (t, dur, why)
        self.arc_pts = []               # (11-05d) camera spots during the current curve piece (the arc fit)
        self.arc_fits = []              # (k ratio, length ratio) per fitted piece
        self.arc_scales = None          # (arc_k_scale, arc_d_scale) the robot said it uses (boot / saved lines)
        self.fp = None                  # (e09) its filter (fused_pose.FusedPose), made by the controller
        self.gap_fwd_t = -1e9
        self.gyro_fix_t = -1e9

    @property
    def new_fw(self):
        return bool(self.fw) and self.fw >= FRESH_FW


# ------------------------------------------------------------------------------------------------ the controller
class Controller:
    def __init__(self, names, rect, out, log, seed=1, lost_mode="reckon", battery_floor=BATTERY_FLOOR,
                 park_sleep=True, fw_override=None, drill=0.0, motion="curves", v_min_avoid=None, edge_rule=None,
                 tracking="lights", attended=False, questions=None, silent_out=False, leads=None, pose_rule=None,
                 keepout_rule=None, tilt_recovery=True, heading_watch=True, pose_source=None, gyro_heading=None,
                 shuffle=True, workspace=None, picture_every_s=None, probe_ahead=False, queue_watch=None,
                 mark_unc=None, start_edge=None, start_mode=None):
        # (e08 Phase A) switches, each with the old behaviour on its other side (CLAUDE.md keep features, switch off):
        #   edge_rule "derived" | "fixed" (EDGE_CC 16 / LOOP_EDGE 9.2 as before); pose_rule "widen" | "wait2s";
        #   keepout_rule "fade" | "permanent"; silent_out False (held, rejoins) | True (out at 120 s as before);
        #   tilt_recovery True (the tries) | False (stands as before); heading_watch True | False
        self.edge_rule = edge_rule or SR.EDGE_RULE
        self.shuffle = bool(shuffle)        # (e10, C4) the corner trap's last rung (False: stands, as before)
        # (e10, C2) probing ahead: a robot may drive slowly into ground nobody has proven yet, its own bottom zone and
        # line sensors guarding it, and prove it (a setting, OFF by default: Nick's "behind a setting")
        self.probe_ahead = bool(probe_ahead)
        # (e10, the 7 Nov run 2 incident) the queue watch, the surface-mark verdict with the uncertainty, the start check
        self.queue_watch_on = QUEUE_WATCH if queue_watch is None else bool(queue_watch)
        self.mark_unc = MARK_UNC if mark_unc is None else bool(mark_unc)
        self.start_edge = START_EDGE if start_edge is None else bool(start_edge)
        self.start_mode = start_mode or START_EDGE_MODE
        # (e10) at most one "unsure near an edge, alone" picture per robot per this many seconds (Claude's default: 60)
        self.picture_every_s = PICTURE_EVERY_S if picture_every_s is None else float(picture_every_s)
        # (e09, Phase B) pose_source "ekf" (one filter per robot: fused_pose.FusedPose) | "blend" (as before);
        # gyro_heading: B3, the heading from the raw gyro rate with its bias taken at every standstill (OFF by default)
        self.pose_source = pose_source or FP.POSE_SOURCE
        self.gyro_heading = FP.GYRO_HEADING if gyro_heading is None else bool(gyro_heading)
        self.fix_fn = None               # host: fix_fn(name, t, x, y) forwards a fused position to the robot (gaps)
        self.never_still = NEVER_STILL   # (e17, Nick 12 Nov) held robots creep (MIRLT_NEVER_STILL=0: they wait)
        self.record_fn = None            # (e17) host: record_fn(name, t, d_cm, travel_cm): a measured correction
        self.pictures = []               # (B4) pictures the host should save: dict(t, robot, why)
        self.camera_proven = True        # (e09) set by the host from its tracking layer: the camera's picture has turned
                                         # the robots the way their own gyros did. Until then a camera spot never sets a
                                         # robot's own pose by itself (no re-seed) nor names "inward" for an unsure robot
        self.pose_rule = pose_rule or SR.POSE_RULE
        self.keepout_rule = keepout_rule or SR.KEEPOUT_RULE
        self.tracking = tracking
        self.attended = bool(attended)
        self.asks = questions            # safety_rules.Questions (attended runs) or None: said and logged only
        self.silent_out = bool(silent_out)
        self.lead_tab = dict(leads or {})   # {robot: {speed: cm}} its stop lead from its calibration (A5)
        self.tilt_recovery = bool(tilt_recovery)
        self.heading_watch = bool(heading_watch)
        # (e10, C2) the proven workspace: a ProvenWorkspace (a tuple of its outer limit, with the grid) in place of the
        # board rectangle; None: the rectangle, as before (the switch)
        self.ws = workspace
        self.rect = workspace if workspace is not None else tuple(float(v) for v in rect)
        self.out_fn, self.log = out, log
        self.rng = random.Random(seed)
        self.seed = seed
        self.menu = Menu(random.Random(seed + 1))
        # (11-05d) the motion style: "curves" (robots on firmware 2026-11-05c+; older ones keep stop-turn) or
        # "stopturn" (everyone as before). Nick: free roam defaults to curves, both always available
        if motion not in ("curves", "stopturn"):
            raise ValueError("motion must be curves or stopturn")
        self.motion_style = motion
        self.cmenu = CurveMenu(random.Random(seed + 2))
        self.v_min = V_MIN_AVOID if v_min_avoid is None else float(v_min_avoid)
        self.R = {n: Robot(n, k) for k, n in enumerate(names)}
        self.lost_mode = lost_mode
        self.floor = battery_floor
        self.park_sleep = park_sleep
        self.fw_override = fw_override
        self.drill = float(drill)       # SIMULATOR ONLY (freeroam.py refuses it on robots): this share of moves is
                                        # planned ignoring the others' reservations, to make the reflex work
        self.t = 0.0
        self.t_start = None
        self.ev_id = 0
        self.last_motion = -1e9
        self.counts = dict(fallback={}, reflex={}, breaks=0, contacts=0, front_stops=0, keepouts=0, backoffs=0,
                           cam_refused=0, dodges=0, pieces=0, loops=0, zeros={})
        self.ended = False
        self.keepouts = []              # discovered: dict(x, y, r, t, robot, src); the planner keeps out for the run
        self.delays = []                # arrival - picture time (this clock), recent fixes
        self.cam_delay, self.cam_delay_hi = CAM_DELAY0, CAM_DELAY0
        self.clock = 0.0                # camera clock minus this clock, applied (0 inside the deadband)
        self.clock_seen = None          # the alignment's own estimate (reported even inside the deadband)
        self.clock_ests = []
        self.clock_min = None           # the camera clock is at least this far ahead (a fix cannot arrive before it
                                        # was taken)
        self.last_clock = self.last_delay = self.last_camlog = -1e9

    # -- I/O ---------------------------------------------------------------------------------------------------
    def out(self, r, text):
        r.last_sent = self.t
        self.out_fn(r.name, text)

    def event(self, r, kind, text):
        self.log("events", dict(t=round(self.t, 2), robot=r.name if r else "-", kind=kind, text=text))

    def send_step(self, r, fmt):
        sid = r.next_id
        r.next_id += 1
        r.steps[sid] = dict(text=fmt % sid, status="queued_local", t_sent=-1e9)
        return sid

    def control(self, r, text, kind):
        r.controls = [c for c in r.controls if c["kind"] != kind]
        r.controls.append(dict(text=text, t_last=self.t, kind=kind))
        self.out(r, text)

    def pump(self, r):
        """Steps go one at a time: the next only after the one before was acked (ids must go up); none while a halt is
        not yet acknowledged (a resent halt arriving after the new step would stop it)."""
        for c in r.controls:
            if self.t - c["t_last"] > RESEND_S:
                c["t_last"] = self.t
                self.out(r, c["text"])
        if any(c["kind"] == "halt" for c in r.controls):
            return
        for sid in sorted(r.steps):
            st = r.steps[sid]
            if st["status"] == "queued_local":
                if any(o["status"] == "sent" for o in r.steps.values()):
                    return
                st["status"], st["t_sent"] = "sent", self.t
                self.out(r, st["text"])
                return
            if st["status"] == "sent" and self.t - st["t_sent"] > RESEND_S:
                st["t_sent"] = self.t
                self.out(r, st["text"])
                return

    # -- inputs ------------------------------------------------------------------------------------------------
    def on_heard(self, name, t):
        r = self.R.get(name)
        if r is None:
            return
        r.heard = t
        if r.silent:
            r.silent = False
            self.event(r, "heard again", "heard again after %.1f s" % (t - (r.silent_t or t)))

    def on_fw(self, name, fw):
        r = self.R.get(name)
        if r is not None:
            r.fw = self.fw_override or fw

    def on_battery(self, name, pct, t):
        r = self.R.get(name)
        if r is None or pct is None:
            return
        r.battery = pct
        if pct != r.bat_logged[0] or t - r.bat_logged[1] > 60.0:
            r.bat_logged = (pct, t)
            self.log("battery", dict(t=round(t, 1), robot=name, pct=pct))

    def on_fix(self, name, t, x, y, th=None, arrival=None, sigma=None, src=None, how=None):
        """A camera fix stamped t (the picture's time on the camera's clock), arrived at `arrival` (this controller's
        clock; None: it counts as arriving now with no delay, as the tests and older callers give it). (11-05c) The
        delay is measured online; the fix is compared with the robot's own pose AT THE PICTURE'S TIME, never now."""
        r = self.R.get(name)
        if r is None:
            return
        if arrival is not None:
            lead = t - arrival
            # (11-05h, home run 5 Nov) not the all-time max: a few odd fixes stamped ahead of their arrival ratcheted
            # it to 0.44 then 1.32 s, the applied clock followed, and every fix read 1.3 s staler than it was (the
            # robots stood "near an edge" and lost). The 90th percentile of the last 300, once there are 50
            self.leads = (getattr(self, "leads", None) or []) + [lead]
            del self.leads[:-300]
            if len(self.leads) >= 50:
                self.clock_min = float(np.percentile(self.leads, 90))
            r.cam_raw.append((t, x, y))
            if len(r.cam_raw) > 200:
                del r.cam_raw[:50]
            t = t - self.clock
            self.delays.append(arrival - t)
            if len(self.delays) > 400:
                del self.delays[:100]
        arr = t if arrival is None else arrival
        r.pc = None
        ages = r.__dict__.setdefault("fix_ages", [])        # (e17, 2) this robot's measured fix age at arrival
        ages.append(max(0.0, arr - t))
        del ages[:-100]
        if how and CAMERA_CORRECT and str(how).startswith(("elimination", "wiggle")):
            return self.correct_from_camera(r, t, x, y, th, arr, how, src)
        oa = self.own_at(r, t)
        if oa is not None and math.hypot(x - oa[0], y - oa[1]) <= REACQUIRE_CM:
            r.agree_t = t                                     # (e17, 6) own and camera agree here (drift record)
        if r.lost is None and oa is not None and r.cam_off is not None:
            off = (x - oa[0], y - oa[1])
            r.jump_n = r.jump_n + 1 if math.dist(off, r.cam_off) > REACQUIRE_MAX_CM else 0
            if r.jump_n >= 3:
                # the camera's spot jumped away from where its own wheels say it went (three fixes in a row): not
                # believed until it agrees
                r.jump_n = 0
                self.start_lost(r, "the camera's spot jumped %.0f cm from its own pose's track" %
                                math.dist(off, r.cam_off))
            elif r.jump_n:
                return                       # one odd fix: skipped
        if r.lost is not None:
            self.reacquire(r, t, x, y, th, arr)
            if r.lost is not None:
                return
        r.cam = (t, x, y, th)
        r.cam_arr = arr
        if self.ws is not None and getattr(self.ws, "limited", False) and self.ws.edge_dist(x, y) >= HALF:
            r.last_in = (x, y)                 # (e13) the last camera fix well inside the readable area
        if r.__dict__.get("hd_drive") is not None and (th is not None or (src or "").split(":")[-1] in ("tags", "")):
            # (e13) the camera reads it again (with a heading): no second click needed
            r.hd_drive = None
            r.place_asked = None
            fn = getattr(self, "place_cancel_fn", None)
            if fn is not None:
                try:
                    fn(r.name)
                except Exception:  # noqa: BLE001
                    pass
            self.event(r, "heading from the camera", "the camera reads it again: the second click is not needed")
        pl = r.__dict__.get("place")
        if pl is not None:
            # (e13) the first camera fix after a click: the click's residual (its own driving since, by its wheels, added)
            r.place = None
            dx = dy = 0.0
            if pl.get("own") is not None and r.own is not None:
                dx, dy = r.own[1] - pl["x"], r.own[2] - pl["y"]
            res = math.hypot(x - (pl["x"] + dx), y - (pl["y"] + dy))
            self.counts["place_residuals"] = self.counts.get("place_residuals", 0) + 1
            self.log("place", dict(t=round(self.t, 2), robot=r.name, how=pl["how"], click_x=round(pl["x"], 2),
                                   click_y=round(pl["y"], 2), cam_x=round(x, 2), cam_y=round(y, 2),
                                   residual_cm=round(res, 2), after_s=round(self.t - pl["t"], 1),
                                   driven_cm=round(math.hypot(dx, dy), 1)))
            pl["driven_cm"] = math.hypot(dx, dy)
            fn = getattr(self, "place_residual_fn", None)
            if fn is not None:
                try:
                    fn(r.name, pl, x, y, res)
                except Exception:  # noqa: BLE001
                    pass
        self.fp_fix(r, t, x, y, th, sigma, src)
        if r.curve and r.move is not None and r.move.get("curve") and r.move["phase"] == "drive":
            r.arc_pts.append((x, y))
        if oa is not None:
            r.cam_off = (x - oa[0], y - oa[1])
            if self.pose_rule == "wait2s":
                self.pose_agree(r, (oa[0], oa[1]), (x, y), th)      # the rule before e08 (kept behind the switch)
            else:
                self.pose_fuse(r, (oa[0], oa[1]), (x, y), th, t_pic=t)
        r.cam_hist.append((t, x, y))
        if len(r.cam_hist) > 80:
            del r.cam_hist[:20]
        if oa is not None and r.lost is None:
            self.heading_check(r, x, y)            # (e08, A5) its own heading against the camera's motion

    def on_follow(self, name, t, state, x, y, th, cur=0, last_done=None, last_failed=None):
        r = self.R.get(name)
        if r is None:
            return
        # a step's own end report can be lost (a WiFi drop: the watchdog stopped it and said so into the gap); the
        # FOLLOW record (10 a second) carries the last done and last failed step ids, so the end is noticed anyway
        if r.move is not None and (last_done is not None or last_failed is not None):
            ids = [i for i in (r.move.get("turn_id"), r.move.get("drive_id")) if i]
            st = {i: r.steps.get(i, {}).get("status") for i in ids}
            ev = None
            if last_failed and last_failed in st and st[last_failed] not in ("done", "failed", "dropped"):
                ev = (last_failed, "failed")
            elif last_done and ids and max(ids) <= last_done and st[max(ids)] not in ("done", "failed", "dropped") \
                    and state == "idle":
                ev = (max(ids), "done")
            if ev is None:
                r.lost_rep = None
            elif getattr(r, "lost_rep", None) is None or r.lost_rep[0] != ev:
                r.lost_rep = (ev, t)                  # the step's own report may still be on its way: 0.5 s
            elif t - r.lost_rep[1] >= 0.5:
                r.lost_rep = None
                self.event(r, "lost report", "step %d %s (its own report was lost: seen in FOLLOW)" % ev)
                self.on_step(name, ev[0], ev[1], "report lost; seen in FOLLOW" if ev[1] == "failed" else "")
        if r.move is not None and r.move.get("curve") and cur and any(q["drive_id"] == cur for q in r.queue):
            # (11-05d) the robot runs a queued piece (its own reports of the last one's end and this one's start lost)
            self.on_step(name, cur, "started")
        if t - getattr(r, "fpose_t", -1e9) < 0.15:
            return          # (11-05c) a record sent before our fpose reached it: the old frame
        prev = r.own
        r.own = (t, x, y, th, state, cur)
        if r.fp is not None and r.fp.started:
            if t - getattr(r, "fpose_t", -1e9) < 0.5:
                r.fp.reset_own_turn()              # its heading was just set (fpose): no turn across it
            else:
                r.fp.own_turn(t, th)               # (e09) the robot's own heading's turn, as a reading
        r.pc = None
        r.own_hist.append((t, x, y, th))
        import observe as OB
        if not (r.move is not None and r.move.get("track_back")):        # (e16) not its own reverse over it
            r.__dict__.setdefault("trail", OB.Trail()).add(x, y, th)  # (11-05h, 4'a) its breadcrumb trail
        if len(r.own_hist) > 200:          # ~20 s at 10 a second: covers a camera delay of several seconds
            del r.own_hist[:50]
        if r.lost is not None and prev is not None:
            d = math.hypot(x - prev[1], y - prev[2])
            r.lost["drive"] += d
            r.lost["turn"] += abs(wrap180(th - prev[3]))
            r.lost["after_turn"] += d
            self.grow_bubble(r)

    def on_step(self, name, sid, state, info=""):
        r = self.R.get(name)
        if r is None:
            return
        if state in ("held", "halted"):
            r.controls = [c for c in r.controls if c["kind"] not in ("halt", "hold")]
            return
        st = r.steps.get(sid)
        if st is not None and st["status"] == "dropped" and state == "started" and self.queue_watch_on and \
                st.get("t_sent", -1e9) > -1e8:
            # (e10, run 2 incident a) a queued piece this controller dropped started on the robot anyway (the drop and
            # the robot's own start crossed): it is not driven blind, it is halted and planned afresh
            self.counts["dropped_started"] = self.counts.get("dropped_started", 0) + 1
            self.halt(r, "a dropped piece %d started" % sid)
            if r.move is not None:
                self.end_move(r)
            return
        if st is None or st["status"] == "dropped":
            return                  # a step this controller already dropped (halted, replaced): its report is old news
        if state in ("queued", "dup") and st["status"] == "sent":
            st["status"] = "acked"
        elif state == "full":
            st["status"] = "queued_local"
        elif state == "bad":
            st["status"] = "failed"
            self.step_over(r, sid, "failed", "bad: " + info)
        elif state == "started":
            st["status"] = "started"
            for q in r.queue:
                if q["drive_id"] == sid and q.get("t_drive") is None:
                    q["t_drive"] = self.t
            # (11-05d) a queued piece started: the one before it is over even if its own report was lost
            while r.move is not None and r.move.get("curve") and any(q["drive_id"] == sid for q in r.queue) and \
                    r.move.get("drive_id") != sid:
                self.event(r, "lost report", "piece %d done (the next piece started; its own report was lost)" %
                           r.move["drive_id"])
                r.steps.get(r.move["drive_id"], {})["status"] = "done"
                self.curve_step_over(r, r.move["drive_id"], "done", "")
            if r.move and sid == r.move.get("drive_id"):
                r.move["phase"] = "drive"
                r.move["t_drive"] = self.t
        elif state in ("done", "failed", "dropped"):
            st["status"] = state
            self.step_over(r, sid, state, info)

    def on_fspd(self, name):
        r = self.R.get(name)
        if r is not None:
            r.controls = [c for c in r.controls if c["kind"] not in ("fspd", "fsteer", "farc")]   # (11-05d) farc too

    # -- where is it -------------------------------------------------------------------------------------------
    def fresh(self, r):
        """(11-05c) Lost = no fix arrived for LOST_AFTER_S, or the newest is older than LOST_AFTER_S beyond the
        measured delay (5 Nov: fixes arrived 0.45 s old, so every 0.15 s gap read as lost)."""
        if r.cam is None:
            return False
        arr = r.cam_arr if r.cam_arr is not None else r.cam[0]
        if STAND_TRUST and self.standing(r):
            return True                      # (e13, Nick 13:34) it has not moved since that fix: the fix still holds
        la = self.lost_after(r)              # (e13) longer only when this robot's own fixes come slower than that
        return self.t - arr < la and self.t - r.cam[0] < la + self.cam_delay_hi

    def standing(self, r):
        """(e13; coordinator from Nick 13:34 "if it finds it and doesn't move it, can't it trust that it knows where it
        is?") True when the robot is not driving and its own wheels and gyro (FOLLOW, current) show it has not moved
        since its last camera fix: that fix still holds and its error sum does not grow while it stands; sparse reads
        (a dark room's slow camera) only confirm it. Switch: MIRLT_STAND_TRUST=0 (every fix ages as before)."""
        if r.cam is None or r.own is None or self.t - r.own[0] > 1.0 or r.mode == "move" or r.move is not None:
            return False
        oa = self.own_at(r, r.cam[0])
        if oa is None:
            return False
        return math.hypot(r.own[1] - oa[0], r.own[2] - oa[1]) < STAND_CM and abs(wrap180(r.own[3] - oa[2])) < STAND_DEG

    def lost_after(self, r):
        """(e13, school 13:1x lights off: the camera fell to ~7 pictures a second) how long with no fix means lost: at
        least LOST_AFTER_S, and never under LOST_GAPS times this robot's longest recent gap between fixes (its last 40),
        so a camera much slower in the dark (under about 2.5 pictures a second) does not call a robot lost between two
        of its pictures; at 7 a second LOST_AFTER_S already covers it."""
        h = r.cam_hist
        if len(h) < 10:
            return LOST_AFTER_S
        hh = h[-41:]
        gaps = sorted(b[0] - a[0] for a, b in zip(hh, hh[1:]) if b[0] > a[0])
        if not gaps:
            return LOST_AFTER_S
        return max(LOST_AFTER_S, LOST_GAPS * gaps[-1])

    def own_at(self, r, t):
        """The robot's own pose (x, y, th) at time t, from its FOLLOW records (interpolated); None if t is older than
        the history kept."""
        h = r.own_hist
        if not h or t < h[0][0] - 0.05:
            return None
        if t >= h[-1][0]:
            return h[-1][1:4]
        k = len(h) - 1
        while k > 0 and h[k - 1][0] > t:
            k -= 1
        if k == 0:
            return h[0][1:4]
        a, b = h[k - 1], h[k]
        f = (t - a[0]) / max(1e-6, b[0] - a[0])
        return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), wrap180(a[3] + f * wrap180(b[3] - a[3])))

    def projected(self, r, t, x, y, th=None):
        """A camera spot taken at t moved on to now by the robot's own motion since then (x, y, th or None);
        None when its own pose does not reach back that far or is not current."""
        if r.own is None or self.t - r.own[0] > 1.0:
            return None
        oa = self.own_at(r, t)
        if oa is None:
            return None
        return (x + r.own[1] - oa[0], y + r.own[2] - oa[1],
                None if th is None else wrap180(th + wrap180(r.own[3] - oa[2])))

    def pos(self, r):
        if r.pc is not None and r.pc[0] == self.t:
            return r.pc[1]
        p = None
        if self.ekf_live(r):
            q = r.fp.pose(self.t)                   # (e09, B2) the filter's estimate now
            p = (q[0], q[1])
            r.pc = (self.t, p)
            return p
        if r.lost is None and r.cam is not None:
            q = self.projected(r, r.cam[0], r.cam[1], r.cam[2])
            p = (q[0], q[1]) if q else (r.cam[1], r.cam[2])
            r.pc_proj = q is not None
        elif r.own is not None:
            p = (r.own[1], r.own[2])
        elif r.cam:
            p = (r.cam[1], r.cam[2])
        r.pc = (self.t, p)
        return p

    # -- camera timing (11-05c) ----------------------------------------------------------------------------------
    def camera_timing(self):
        if self.t - self.last_delay >= 1.0 and len(self.delays) >= 10:
            self.last_delay = self.t
            d = np.array(self.delays[-150:])
            self.cam_delay = float(np.median(d))
            self.cam_delay_hi = float(min(3.0, max(0.0, np.percentile(d, 90)) + 0.1))
        if self.t - self.last_clock >= CLOCK_EVERY_S:
            self.last_clock = self.t
            self.align_clock()
        if self.t - self.last_camlog >= 30.0 and self.delays:
            self.last_camlog = self.t
            self.log("camera", dict(t=round(self.t, 2), delay_med_s=round(self.cam_delay, 3),
                                    delay_p90_s=round(self.cam_delay_hi - 0.1, 3), lost_after_s=round(
                                        LOST_AFTER_S + self.cam_delay_hi, 2), clock_applied_s=round(self.clock, 3),
                                    clock_seen_s=round(self.clock_seen, 3) if self.clock_seen is not None else "",
                                    n=len(self.delays)))

    def align_clock(self):
        """Which time on this clock the camera's stamps mean: the shift that makes the camera's track move as the
        robots' own poses moved (displacements over ~0.5 s, the last 10 s, robots that drove). Applied only outside
        CLOCK_DEADBAND_S; never so that a fix would arrive before it was taken."""
        found = []
        lags = np.arange(-3.0, 3.001, 0.02)
        for r in self.R.values():
            c = np.array([q for q in r.cam_raw if q[0] > r.cam_raw[-1][0] - 10.0]) if r.cam_raw else None
            o = np.array(r.own_hist) if len(r.own_hist) > 10 else None
            if c is None or o is None or len(c) < 20:
                continue
            # speeds over ~0.5 s (camera vs own): starts and stops line up; the own pose's pulls towards the fixes
            # (the firmware's fusion) shift positions, not when it starts and stops
            i = np.arange(0, len(c) - 6, 2)
            j = i + 5
            dt = np.maximum(0.05, c[j, 0] - c[i, 0])
            sc = np.hypot(*(c[j, 1:3] - c[i, 1:3]).T) / dt
            if sc.max() - sc.min() < 4.0 or (r.curve and sc.min() > 1.0):
                continue                     # no start or stop in the window: nothing to line up (11-05d: a curve
                                             # robot's speed changes between pieces lined up wrongly, -0.35 s)
            errs = []
            for L in lags:
                ts = c[:, 0] - L
                ok = (ts >= o[0, 0]) & (ts <= o[-1, 0])
                m = ok[i] & ok[j]
                if m.sum() < 8:
                    errs.append(np.inf)
                    continue
                ox, oy = np.interp(ts, o[:, 0], o[:, 1]), np.interp(ts, o[:, 0], o[:, 2])
                so = np.hypot(ox[j] - ox[i], oy[j] - oy[i]) / dt
                errs.append(float(np.mean(np.abs(sc - so)[m])))
            e = np.array(errs)
            fin = e[np.isfinite(e)]
            # only a clear best (a robot that changed speed or direction): a steady drive fits every shift alike
            if len(fin) > 20 and fin.min() < 0.5 * np.median(fin):
                found.append(float(lags[int(np.argmin(e))]))
        if not found:
            return
        self.clock_ests.append(float(np.median(found)))
        del self.clock_ests[:-9]
        est = float(np.median(self.clock_ests))
        self.clock_seen = est
        # applied only when the last five agree (spread under 0.2 s) and lie outside the deadband
        last = self.clock_ests[-5:]
        steady = len(last) == 5 and max(last) - min(last) < 0.2
        new = est if (steady and abs(est) >= CLOCK_DEADBAND_S) else (self.clock if not steady else 0.0)
        if self.clock_min is not None:
            new = max(new, self.clock_min - 0.02) if self.clock_min > 0.02 else new
        if abs(new - self.clock) > 0.05:
            self.delays = [d + new - self.clock for d in self.delays]     # the same fixes, read on the new clock
            self.cam_delay_hi += new - self.clock
            for r in self.R.values():
                if r.cam is not None:
                    r.cam = (r.cam[0] - (new - self.clock),) + tuple(r.cam[1:])
            self.event(None, "camera clock", "the camera's time stamps are %+.2f s from this clock (from the robots' "
                       "own tracks); fixes are read %s" % (est, "with that shift" if new else "as they are (within "
                                                            "%.2f s)" % CLOCK_DEADBAND_S))
            self.clock = new

    def heading(self, r):
        if self.gyro_heading and self.ekf_live(r) and r.fp.gh.heading_deg() is not None:
            return r.fp.gh.heading_deg()          # (e09, B3) the raw gyro's heading (the robot is told it too)
        if r.own is not None:
            return r.own[3]
        return r.cam[3] if r.cam and r.cam[3] is not None else 0.0

    def margin0(self, r):
        return PLAN_M0 + (r.bubble if r.lost is not None else 0.0) + (3.0 if r.silent else 0.0)

    # -- dead reckoning --------------------------------------------------------------------------------------------
    def grow_bubble(self, r):
        if self.ekf_live(r):
            # (e09, B2) the filter's own uncertainty, grown by its wheels and gyro since the camera last saw it
            r.bubble = max(BLIND_BASE, r.fp.unc_cm(self.t))
            return
        L = r.lost
        lat = L["after_turn"] * math.sin(math.radians(BLIND_TURN_FRAC * L["turn"]))
        r.bubble = L.get("base", BLIND_BASE) + BLIND_PER_CM * L["drive"] + lat     # (e13) base: a click's accuracy

    def start_lost(self, r, why="camera lost it"):
        p = self.pos(r)
        r.lost = dict(t0=self.t, drive=0.0, turn=0.0, after_turn=0.0, at=p, n_ok=0, last=None, chain=[])
        r.pc = None
        r.bubble = BLIND_BASE
        self.event(r, "lost", "%s at %.1f, %.1f: dead reckoning, bubble %.1f cm" % (why, p[0], p[1], r.bubble)
                   if p else why)
        if self.lost_mode == "hold" and r.mode == "move":
            r.lost_hold = True
            self.halt(r, "lost (hold mode)")
            self.trophy_break(r, "lost by the camera (cautious hold switch)")

    def reacquire(self, r, t, x, y, th, arr=None):
        """(11-05c) The fix is compared with the robot's own pose at the picture's time t. A spot further off than
        REACQUIRE_MAX_CM is believed only once the camera's spot has moved as the robot's own pose moved (offset
        unchanged within 3 cm) over VERIFY_CM of its own travel; a camera on something else (a stationary object, the
        other robot) never passes. The pose sent (fpose) is the camera's spot moved on to now by its own motion."""
        L = r.lost
        arr = t if arr is None else arr
        oa = self.own_at(r, t)
        if oa is None and r.own is not None:
            oa = r.own[1:4]
        jump = math.hypot(x - oa[0], y - oa[1]) if oa else 0.0
        if jump > REACQUIRE_MAX_CM and oa is not None:
            off = (x - oa[0], y - oa[1])
            ch = L.setdefault("chain", [])
            if ch and math.dist(off, ch[0][1]) > 3.0:
                if self.t - r.disagree_t > 5.0:
                    r.disagree_t = self.t
                    self.counts["cam_refused"] += 1
                    self.event(r, "camera disagrees", "the camera's spot %.1f, %.1f (%.0f cm from its own pose) did "
                               "not move as its wheels did: not believed, dead reckoning on" % (x, y, jump))
                ch.clear()
            ch.append(((oa[0], oa[1]), off, t))
            travel = sum(math.dist(ch[k][0], ch[k + 1][0]) for k in range(len(ch) - 1))
            # a robot that stands still cannot show it: a steady spot for VERIFY_STILL_S is believed unless another
            # robot is (or is believed to be) there (5 Nov: the camera put Center on Right's spot)
            span = ch[-1][2] - ch[0][2]
            taken = [o.name for o in self.R.values() if o is not r and self.pos(o) is not None and
                     math.dist(self.pos(o), (x, y)) < 2 * HALF + 2.0]
            if len(ch) < 3 or (travel < VERIFY_CM and (span < VERIFY_STILL_S or len(ch) < 10 or taken)):
                if taken and span >= VERIFY_STILL_S and self.t - r.disagree_t > 5.0:
                    r.disagree_t = self.t
                    self.counts["cam_refused"] += 1
                    self.event(r, "camera disagrees", "the camera's spot %.1f, %.1f is where %s is: not believed"
                               % (x, y, taken[0]))
                return
        within = jump <= r.bubble
        self.log("lost", dict(robot=r.name, t0=round(L["t0"], 2), t1=round(arr, 2), dur_s=round(arr - L["t0"], 2),
                              blind_cm=round(L["drive"], 1), blind_deg=round(L["turn"], 1),
                              bubble_cm=round(r.bubble, 2), jump_cm=round(jump, 2), within=int(within),
                              x=round(x, 1), y=round(y, 1), fix_age_s=round(arr - t, 2)))
        if L["drive"] >= 5.0:
            self.drift_add(r, jump, L["drive"], "seen again")        # (e17, 6)
        self.event(r, "seen again", "after %.1f s, %.1f cm and %.0f deg blind: odometry was %.1f cm off at the "
                   "picture's time (bubble %.1f; the fix was %.2f s old)"
                   % (arr - L["t0"], L["drive"], L["turn"], jump, r.bubble, arr - t))
        if jump > REACQUIRE_CM:
            q = self.projected(r, t, x, y, th)
            if q is None:                       # its own pose does not reach back: the fix as it is (old behaviour)
                q = (x, y, th)
            h = q[2] if q[2] is not None else (r.own[3] if r.own else 0.0)
            self.out(r, "fpose %.2f %.2f %.2f 1.0 3.0" % (q[0], q[1], h))
            if r.own is not None:               # its own track from now on is in the new frame: shift the history
                dx, dy = q[0] - r.own[1], q[1] - r.own[2]
                r.own_hist = [(a, b + dx, c + dy, d) for a, b, c, d in r.own_hist]
                r.own = (r.own[0], q[0], q[1], h) + tuple(r.own[4:])
            r.fpose_t = self.t
        r.lost, r.bubble = None, 0.0
        r.pc = None
        if self.ekf_live(r):
            if jump > REACQUIRE_CM:
                q = self.projected(r, t, x, y, th) or (x, y, th)
                r.fp.reset(self.t, q[0], q[1], None, 1.0)
            else:
                r.fp.fix(t, x, y, th, None, "camera", now=self.t)
        if r.lost_hold:
            r.lost_hold = False
        if r.stopped_hard and (r.adj is None or r.adj["kind"] != "stop"):
            self.close_break(r)
        r.cam = (t, x, y, th)
        r.cam_arr = arr
        oa2 = self.own_at(r, t)
        r.cam_off = (x - oa2[0], y - oa2[1]) if oa2 else None

    # -- (e17) observe, then trust: the camera's corrections -------------------------------------------------------
    def own_path(self, r, ta, tb):
        """How far its own pose went between ta and tb (cm, along its FOLLOW records)."""
        d, prev = 0.0, None
        for q in r.own_hist:
            if q[0] < ta:
                prev = q
                continue
            if q[0] > tb:
                break
            if prev is not None:
                d += math.hypot(q[1] - prev[1], q[2] - prev[2])
            prev = q
        return d

    def turn_rate(self, r, win=0.5):
        """Its own heading's turn rate over the last win s (deg/s), from its FOLLOW records."""
        h = [q for q in r.own_hist if q[0] >= self.t - win]
        if len(h) < 2 or h[-1][0] - h[0][0] < 0.05:
            return 0.0
        return abs(wrap180(h[-1][3] - h[0][3])) / (h[-1][0] - h[0][0])

    def drift_add(self, r, d_cm, travel_cm, why):
        """(e17, 6) A measured correction: its own pose was d_cm off the camera after travel_cm driven since they last
        agreed. The robot's own drift record (per cm), used by the late-fix rule; also given to the tracking layer."""
        rec = r.__dict__.setdefault("drift_rec", [])
        rec.append((self.t, float(d_cm), float(travel_cm), why))
        del rec[:-40]
        fn = getattr(self, "record_fn", None)
        if fn is not None:
            try:
                fn(r.name, self.t, d_cm, travel_cm)
            except Exception:  # noqa: BLE001
                pass

    def drift_rate(self, r):
        """(cm of own-pose error per cm driven at its 90th percentile, corrections used); the measured prior
        BLIND_PER_CM (29 Oct, 22,000 stretches) until LATE_REC_MIN corrections after 5 cm or more of driving."""
        rec = [q for q in r.__dict__.get("drift_rec", []) if q[2] >= 5.0 and self.t - q[0] <= 900.0]
        if len(rec) < LATE_REC_MIN:
            return BLIND_PER_CM, len(rec)
        v = sorted(q[1] / q[2] for q in rec)
        return min(1.0, max(BLIND_PER_CM, v[min(len(v) - 1, int(0.9 * len(v)))])), len(rec)

    def send_fpose(self, r, t_pic, x, y, th, sig=1.0, sig_th=3.0, why="re-seed"):
        """(e17, 1e) Set the robot's pose from a camera spot taken at t_pic: moved on to now by its own motion since
        (projected; before e17 the re-seed used the picture's spot as it was, ~0.5-1 s old) and never while it turns
        (the Pi's copy of its heading is ~0.3 s old by the time the fpose lands: 13-15 deg median jump when one landed
        mid-turn in the 11 Nov home runs; at most RESEED_WAIT_MAX_S of waiting). MIRLT_RESEED_TURN=0: at once, as
        before (the picture's spot, the heading now)."""
        if not RESEED_TURN:
            h = th if th is not None else self.heading(r)
            self.out(r, "fpose %.2f %.2f %.2f %.2f %.2f" % (x, y, h, sig, sig_th))
            return True
        if self.turn_rate(r) > RESEED_TURN_DPS or (r.move is not None and r.move.get("phase") == "turn" and
                                                    not r.move.get("curve")):
            pend = r.__dict__.get("fpose_pending")
            r.fpose_pending = dict(t_pic=t_pic, x=x, y=y, th=th, sig=sig, sig_th=sig_th, why=why,
                                   t0=pend["t0"] if pend else self.t)
            if self.t - r.fpose_pending["t0"] < RESEED_WAIT_MAX_S:
                self.counts["reseeds_deferred"] = self.counts.get("reseeds_deferred", 0) + (0 if pend else 1)
                return False
        r.fpose_pending = None
        q = self.projected(r, t_pic, x, y, th) or (x, y, th)
        h = q[2] if q[2] is not None else self.heading(r)
        self.out(r, "fpose %.2f %.2f %.2f %.2f %.2f" % (q[0], q[1], h, sig, sig_th))
        r.fpose_t = self.t
        return True

    def fpose_pending_watch(self, r):
        pend = r.__dict__.get("fpose_pending")
        if pend is not None:
            self.send_fpose(r, pend["t_pic"], pend["x"], pend["y"], pend["th"], pend["sig"], pend["sig_th"],
                            pend["why"])

    def correct_from_camera(self, r, t, x, y, th, arr, how, src=None):
        """(e17, 1d/1e; home 11 Nov: Left held 84 min 48 cm from where the camera saw it standing) The tracking layer
        named this sighting by elimination (a steady sighting nobody else can claim, this robot standing and
        unaccounted) or by its wiggle: the evidence wins. Its pose is set from the camera at once (fpose: the robot's
        own filter would refuse a far fix), its filter re-seeded, lost / doubt cleared. Said "CORRECTED", never
        "moved in": its wheels did not move. A large correction also asks the camera to confirm its heading."""
        oa = self.own_at(r, t)
        if oa is None and r.own is not None:
            oa = r.own[1:4]
        d = math.hypot(x - oa[0], y - oa[1]) if oa else 0.0
        travel = self.own_path(r, r.__dict__.get("agree_t", t - 60.0), t)
        if r.lost is not None:
            travel = max(travel, r.lost.get("drive", 0.0))
        self.drift_add(r, d, travel, "camera correction (%s)" % how)
        self.send_fpose(r, t, x, y, None, 1.0, 3.0, "correction")
        if r.own is not None:                   # its own track from now on is in the new frame: shift the history
            q = self.projected(r, t, x, y, None) or (x, y, None)
            dx, dy = q[0] - r.own[1], q[1] - r.own[2]
            r.own_hist = [(a, b + dx, c + dy, e) for a, b, c, e in r.own_hist]
            r.own = (r.own[0], q[0], q[1]) + tuple(r.own[3:])
        r.fpose_t = self.t
        was_lost = r.lost is not None
        r.lost, r.bubble, r.lost_hold, r.jump_n = None, 0.0, False, 0
        r.pc = None
        if r.fp is not None and r.fp.started:
            r.fp.reset(self.t, x, y, None, 1.0)
        r.__dict__.pop("pfuse", None)           # the old disagreement is settled by the correction
        r.__dict__.pop("pagree", None)
        r.cam = (t, x, y, th)
        r.cam_arr = arr
        r.cam_off = (0.0, 0.0)
        r.cam_hist.append((t, x, y))
        r.agree_t = t
        r.corrected = dict(t=self.t, d_cm=d, x=x, y=y, how=how)
        if r.__dict__.get("place_asked") is not None or r.__dict__.get("hd_drive") is not None:
            r.hd_drive = None
            r.place_asked = None
            fn = getattr(self, "place_cancel_fn", None)
            if fn is not None:
                try:
                    fn(r.name)
                except Exception:  # noqa: BLE001
                    pass
        if HEADING_CONFIRM and d > REACQUIRE_MAX_CM:
            r.hd_confirm = dict(t=self.t, why="its own pose was %.0f cm off" % d, tries=0, stage=None)
        if r.stopped_hard and (r.adj is None or r.adj["kind"] != "stop"):
            self.close_break(r)
        self.counts["camera_corrections"] = self.counts.get("camera_corrections", 0) + 1
        self.log("corrections", dict(t=round(self.t, 2), robot=r.name, how=how, d_cm=round(d, 1),
                                     travel_cm=round(travel, 1), x=round(x, 1), y=round(y, 1),
                                     own_x=round(oa[0], 1) if oa else "", own_y=round(oa[1], 1) if oa else "",
                                     was_lost=int(was_lost), unsure=int(r.__dict__.get("unsure") is not None)))
        self.event(r, "CORRECTED", "the camera sees %s standing at %.0f, %.0f (%s); its own pose was %.1f cm off and "
                   "its wheels did not move: its pose is set from the camera (a correction, not a move)" % (
                       r.name, x, y, "the only steady sighting nobody else can claim" if how == "elimination" else
                       "the sighting that turned when it wiggled", d))

    def unaccounted_why(self, r):
        """(e17) Why the run cannot account for r now (lost, held, unsure near an edge, needs a hand), or None: the
        tracking layer's elimination looks for its steady sighting."""
        if r.mode in ("out", "gone", "start"):
            return None
        if r.lost is not None:
            return "lost"
        if getattr(r, "held", None):
            return "held: %s" % r.held.get("kind")
        if r.__dict__.get("unsure") is not None:
            return "unsure near an edge"
        if r.name in (self.__dict__.get("need_hand") or {}):
            return "needs a hand"
        return None

    def on_elim_revoked(self, ev):
        """(e17) The layer took back an elimination (the spot stayed while r drove): r is lost again (dead reckoning,
        the blind rule) until the camera has it."""
        r = self.R.get(ev.get("robot"))
        if r is None:
            return
        self.counts["corrections_revoked"] = self.counts.get("corrections_revoked", 0) + 1
        self.event(r, "correction revoked", "the camera spot at %.0f, %.0f stayed while %s drove: it was not %s" % (
            ev["x"], ev["y"], r.name, r.name))
        if r.lost is None:
            self.start_lost(r, "its camera spot was not it")

    # -- (e17, 2) no driving blind on late fixes ------------------------------------------------------------------
    def fix_delay(self, r):
        a = sorted(r.__dict__.get("fix_ages", [])[-40:])
        if len(a) < 5:
            return max(0.2, self.cam_delay_hi)
        return a[min(len(a) - 1, int(0.9 * len(a)))]

    def late_fix_watch(self, r):
        """(e17, 2; home 11 Nov 11:39: Left drove ~50 cm on its own guess while the fixes arrived ~1 s late) Every tick
        while it drives: by the time the next fix lands (its own measured fix delay, 90th percentile) its own pose may
        be off by BLIND_BASE + its drift rate (measured, per cm) x the distance driven since the newest picture. When
        that could pass the room it has (the nearest edge less its half diagonal, the nearest robot less both), it
        slows to what keeps it inside, and under LATE_SLOW_MIN of its speed it waits (fspd 0) for a fix; the next
        piece is not queued meanwhile. Back to full speed when a fix makes the sum fit again."""
        late = r.__dict__.get("late")
        mv = r.move
        ok = LATE_FIX and r.mode == "move" and mv is not None and r.lost is None and r.cam is not None and \
            not r.silent and r.new_fw and mv.get("t_drive") is not None and mv.get("phase") == "drive" and \
            not str(mv.get("kind", "")).startswith(("wiggle", "nudge", "hdconfirm", "unsure", "recover", "probe", "creep"))
        if not ok or r.adj is not None:
            if late is not None and (mv is None or r.mode != "move"):
                r.late = None
            elif late is not None and not ok and r.adj is None:
                # the drive it slowed is over (a coast, a turn, a halt): its next part runs at its own speed
                r.late = None
                r.scale = 1.0
                self.control(r, "fspd 1.00", "fspd")
            return
        p = self.pos(r)
        if p is None:
            return
        v = max(0.5, mv["v"])
        delay = self.fix_delay(r)
        d_since = self.own_path(r, r.cam[0], self.t)
        rate, n_rec = self.drift_rate(r)
        h = math.radians(self.heading(r))
        ahead = (p[0] + math.cos(h) * v * delay, p[1] + math.sin(h) * v * delay)
        room = min(edge_dist(self.rect, *p), edge_dist(self.rect, *ahead)) - HALF
        for o in self.R.values():
            if o is r or o.mode in ("gone",):
                continue
            po = self.pos(o)
            if po is not None:
                room = min(room, math.dist(p, po) - 2 * HALF - REFLEX_GAP)
        err_full = BLIND_BASE + rate * (d_since + v * delay)
        if late is None and err_full <= room:
            return
        if late is not None and err_full <= 0.8 * room:
            r.late = None
            r.scale = 1.0
            self.control(r, "fspd 1.00", "fspd")
            self.counts["late_resumed"] = self.counts.get("late_resumed", 0) + 1
            self.log("late_fix", dict(t=round(self.t, 2), robot=r.name, what="full speed", err_cm=round(err_full, 1),
                                      room_cm=round(room, 1), delay_s=round(delay, 2), rate=round(rate, 3)))
            return
        allowed = (room - BLIND_BASE) / max(1e-6, rate) - d_since
        scale = max(0.0, min(1.0, allowed / max(1e-6, v * delay)))
        if scale < LATE_SLOW_MIN:
            scale = 0.0
        if self.never_still and late is not None and self.t - late.get("t0", late["t"]) >= self.still_pause(r):
            # (e17, Nick 12 Nov) a pause, not a stop: on at its slowest (never under the firmware's 2 cm/s floor)
            scale = max(scale, LATE_SLOW_MIN, min(1.0, CREEP_V_MIN / v))
        if late is not None and abs(scale - late["scale"]) < 0.1:
            return
        if late is None:
            self.counts["late_slowed" if scale > 0 else "late_waited"] = self.counts.get(
                "late_slowed" if scale > 0 else "late_waited", 0) + 1
            self.event(r, "late fixes", "%s: fixes arrive %.2f s late and its own pose may be %.1f cm off by then "
                       "(%.1f cm driven since the newest picture, drift %.2f cm per cm%s); room %.1f cm" % (
                           "waits for a fix" if scale == 0 else "slows to %.0f %%" % (100 * scale), delay, err_full,
                           d_since, rate, " measured over %d corrections" % n_rec if n_rec >= LATE_REC_MIN else
                           " (prior)", room))
        r.late = dict(t=self.t, scale=scale, t0=late.get("t0", self.t) if late else self.t)
        r.scale = scale
        self.control(r, "fspd %.2f" % scale, "fspd")
        self.why_zero(r, "late fixes: waits for the camera") if scale == 0 else None
        self.log("late_fix", dict(t=round(self.t, 2), robot=r.name, what="scale %.2f" % scale,
                                  err_cm=round(err_full, 1), room_cm=round(room, 1), delay_s=round(delay, 2),
                                  rate=round(rate, 3), d_since_cm=round(d_since, 1)))

    # -- (e17, Nick 12 Nov) never still: creep toward safety and being found --------------------------------------
    def still_pause(self, r):
        """The longest a held robot stands between creeps: one fix delay (90th percentile, measured) for a picture
        taken after it stopped, plus half a second for the camera to settle. Derived; no DOE."""
        return self.fix_delay(r) + 0.5

    def creep_target(self, r, p):
        """Its best guess of safety AND being found: back into the readable area (its last fix well inside, or the
        nearest readable spot) when it is outside it; else the middle of the board (furthest from every edge, where
        every camera sees)."""
        ws = self.ws
        if ws is not None and getattr(ws, "limited", False) and ws.edge_dist(*p) < 0.0:
            t_ = r.__dict__.get("last_in") or ws.nearest_allowed(p[0], p[1], HALF)
            if t_ is not None:
                return tuple(t_[:2]), "back into the area the camera reads"
        b = getattr(self.rect, "board", None) or self.rect
        x0, y0, x1, y1 = (float(v) for v in tuple(b)[:4])
        mid = ((x0 + x1) / 2.0, (y0 + y1) / 2.0)
        if edge_dist(self.rect, *p) >= EDGE_CC + CREEP_MAX_CM:
            # already well clear of every edge: safety now means room from the others (several robots creeping to
            # the middle would crowd it): away from the nearest robot, kept clear of the edges
            near = [(math.dist(p, self.pos(o)), self.pos(o)) for o in self.R.values()
                    if o is not r and self.pos(o) is not None and o.mode != "gone"]
            if near:
                d, q = min(near)
                if d > 0.5:
                    tx = p[0] + (p[0] - q[0]) / d * 20.0
                    ty = p[1] + (p[1] - q[1]) / d * 20.0
                    m = EDGE_CC + CREEP_MAX_CM
                    return (min(max(tx, x0 + m), x1 - m), min(max(ty, y0 + m), y1 - m)), \
                        "away from %s, clear of the edges" % min(
                            (o for o in self.R.values() if o is not r and self.pos(o) == q), key=lambda o: 0).name
        return mid, "toward the middle of the board"

    def creep_step(self, r, why):
        """(e17, Nick 12 Nov: "Movement should never settle at 0 ... It's okay if they fall") A small slow move toward
        creep_target from its best position (the camera's when fresh, else its fused / own), step and speed scaled down
        by its uncertainty (UNC0 / (UNC0 + unc)). Reverses when the target is behind it (no turn), else turns on the
        spot first. Keeps the reflex gap to the others' plans (other bearings tried, 45 and 90 deg off); no edge rule
        (a fall is allowed while experimenting). Returns True when a move was sent."""
        p = (r.cam[1], r.cam[2]) if (r.cam is not None and r.lost is None and self.fresh(r)) else self.pos(r)
        if p is None or r.silent:
            return False
        h = self.heading(r)
        tgt, words = self.creep_target(r, p)
        dist = math.dist(p, tgt)
        unc = r.bubble if r.lost is not None else self.unc(r)
        f = CREEP_UNC0_CM / (CREEP_UNC0_CM + max(0.0, unc))
        L = max(CREEP_MIN_CM, min(CREEP_MAX_CM * f, max(CREEP_MIN_CM, dist)))
        v = max(CREEP_V_MIN, BACK_V * f)
        brg = math.degrees(math.atan2(tgt[1] - p[1], tgt[0] - p[0])) if dist > 0.5 else h
        near = [(math.dist(p, self.pos(o)), self.pos(o)) for o in self.R.values()
                if o is not r and self.pos(o) is not None and o.mode != "gone"]
        away = None
        if near:
            d0, q0 = min(near)
            if d0 > 0.5:
                away = math.degrees(math.atan2(p[1] - q0[1], p[0] - q0[0]))
        tries = [(brg, off) for off in (0.0, 45.0, -45.0, 90.0, -90.0, 135.0, -135.0, 180.0)]
        if away is not None:
            tries.append((away, None))         # last: straight away from the nearest robot, even inside a reservation
        for base, off in tries:
            b_ = wrap180(base + (off or 0.0))
            back = abs(wrap180(b_ - (h + 180.0))) <= 60.0
            turn = 0.0 if back else wrap180(b_ - h)
            hd = wrap180(h + 180.0) if back else wrap180(h + turn)
            tr = Traj.move(self.t + 0.2 + (turn_s(turn) if turn else 0.0), p, hd, 0.0, L, v, m0=self.margin0(r))
            if off is None:
                # its start is already inside another robot's reservation (every bearing "blocked"): moving away
                # widens the gap to every robot, so it goes (the reflex still watches)
                end = tr.end
                if math.dist(end, q0) < d0 + 0.5 * L or any(math.dist(end, q) < dq - 0.1 for dq, q in near):
                    continue
            elif any(clearance_profile(tr, o.traj, now=self.t)[-1] < REFLEX_CC for o in self.others(r)):
                continue
            kind = "creep back" if back else "creep"
            mv = dict(kind=kind, cell=None, turn=turn, L=L, v=v, t0=self.t, p0=p, h0=h, h1=wrap180(h + turn),
                      end=tr.end, phase="turn", n=r.n_moves + 1, drive_id=None, turn_id=None, t_drive=None,
                      adj_scale=1.0, rev=back)
            if abs(turn) >= 3.0:
                mv["turn_id"] = self.send_step(r, "fs %%d T %.2f -" % wrap180(h + turn))
            mv["drive_id"] = self.send_step(r, "fs %%d D %.2f %.2f %.2f %.2f %.2f %.2f %d -" % (
                p[0], p[1], tr.end[0], tr.end[1], wrap180(h + turn), v, 1 if back else 0))
            r.move, r.traj, r.mode, r.queue = mv, tr, "move", []
            r.n_moves += 1
            r.creep_n = r.__dict__.get("creep_n", 0) + 1
            self.counts["creeps"] = self.counts.get("creeps", 0) + 1
            last = r.__dict__.get("creep_said", -1e9)
            if self.t - last >= 30.0:
                r.creep_said = self.t
                self.event(r, "creeps", "%s: %.0f cm at %.1f cm/s %s (%s; its doubt %.1f cm)" % (
                    why, L, v, words, "reversing" if back else "turning %.0f deg first" % turn, unc))
            self.log("creep", dict(t=round(self.t, 2), robot=r.name, why=why, L_cm=round(L, 1), v=round(v, 1),
                                   unc_cm=round(unc, 1), x=round(p[0], 1), y=round(p[1], 1), tx=round(tgt[0], 1),
                                   ty=round(tgt[1], 1), off_deg=off if off is not None else "away", back=int(back)))
            self.why_zero(r, "creep (%s)" % why)
            self.pump(r)
            return True
        r.busy_until = self.t + 0.5                     # every way is another robot's: a brief pause
        return False

    def never_still_watch(self, r):
        """(e17) Every tick: a robot that has stood idle longer than still_pause (held at an edge, unsure, needing a
        hand, waiting for a heading check's room, put down at an edge, tilted with no way back, outside the readable
        area with no return found) creeps. Not while it waits for a heading check's measurement or a short pause the
        run asked for itself under that pause; never one that is silent, parked, out, or at its battery floor."""
        if not self.never_still or r.mode != "idle" or r.move is not None:
            r.still_since = None
            return
        if r.__dict__.get("still_since") is None:
            r.still_since = self.t
            return
        if r.silent or getattr(r, "held", None) or (r.battery is not None and r.battery <= self.floor) or \
                self.pos(r) is None:
            return
        hc = r.__dict__.get("hd_confirm")
        if hc is not None and hc.get("stage") == "measure":
            return
        if self.t - r.still_since < self.still_pause(r):
            return
        why = self.edge_stuck_why(r) or self.unaccounted_why(r) or (
            "waits for room to check its heading" if hc is not None else "stood still")
        if self.creep_step(r, why):
            r.still_since = None

    # -- (e17, 2) a camera-confirmed heading after a heading re-set --------------------------------------------------
    def heading_confirm_step(self, r):
        """After an off-plan heading re-set (or a large correction), the next move is a short slow straight crawl
        whose direction the camera measures: within HD_CONFIRM_DEG of its own heading = confirmed; else its heading is
        set from the camera (standing: nothing turns under the fpose) and checked again. The crawl's length is derived
        from the camera's noise on this robot (sqrt 2 x sigma / sqrt n / tan(HD_CONFIRM_DEG / 2), n fixes averaged at
        each end), 6-12 cm, and made only where that length in ANY direction keeps the derived edge clearance and the
        other robots' gap (its heading is what is in doubt). No room: the next move is planned as before, said."""
        hc = r.hd_confirm
        p = self.pos(r)
        if hc.get("stage") is None:
            if not self.fresh(r) or r.cam is None:
                r.busy_until = self.t + 0.3
                return
            c0 = [q for q in r.cam_hist if q[0] >= self.t - 1.5 and q[0] >= hc.get("t_idle", hc["t"])]
            if len(c0) < 2:
                hc.setdefault("t_idle", self.t)
                r.busy_until = self.t + 0.3
                return
            sx = sum(q[1] for q in c0) / len(c0)
            sy = sum(q[2] for q in c0) / len(c0)
            sd = math.sqrt(sum((q[1] - sx) ** 2 + (q[2] - sy) ** 2 for q in c0) / len(c0))
            sig = max(0.3, sd)
            L = min(HD_CONFIRM_MAX_CM, max(6.0, math.sqrt(2.0) * sig / math.sqrt(3.0) /
                                           math.tan(math.radians(HD_CONFIRM_DEG / 2.0))))
            need = self.need_cm(r, BACK_V, True) + L
            crowd = any(math.dist(self.pos(o), p) < PLAN_CC + L for o in self.R.values()
                        if o is not r and self.pos(o) is not None and o.mode != "gone")
            if crowd:
                # another robot within reach of a crawl in some direction: it waits for room (they roam on); a move
                # on a doubtful heading next to a robot is how 08:48 pushed Right off the board
                if not hc.get("said_wait"):
                    hc["said_wait"] = True
                    self.counts["hd_confirm_waits"] = self.counts.get("hd_confirm_waits", 0) + 1
                    self.event(r, "heading check waits", "its heading is in doubt (%s) and a robot is within %.0f cm: "
                               "it stands until there is room for a %.0f cm check" % (hc["why"], PLAN_CC + L, L))
                r.busy_until = self.t + 1.0
                return
            if edge_dist(self.rect, *p) < need:
                r.hd_confirm = None
                self.counts["hd_confirm_no_room"] = self.counts.get("hd_confirm_no_room", 0) + 1
                self.event(r, "heading not confirmed", "no room for a %.0f cm heading check here (near an edge): the "
                           "next move is planned as before (the edge rule with its doubt)" % L)
                return
            h = self.heading(r)
            tr = Traj.move(self.t + 0.2, p, h, 0.0, L, BACK_V, m0=self.margin0(r) + L)
            mv = dict(kind="hdconfirm", cell=None, turn=0.0, L=L, v=BACK_V, t0=self.t, p0=p, h0=h, h1=h,
                      end=tr.end, phase="turn", n=r.n_moves + 1, drive_id=None, turn_id=None, t_drive=None,
                      adj_scale=1.0, rev=False)
            mv["drive_id"] = self.send_step(r, "fs %%d D %.2f %.2f %.2f %.2f %.2f %.2f 0 -" % (
                p[0], p[1], tr.end[0], tr.end[1], h, BACK_V))
            hc.update(stage="crawl", c0=(sx, sy), L=L, h_own=h)
            r.move, r.traj, r.mode = mv, tr, "move"
            r.n_moves += 1
            self.counts["hd_confirm_crawls"] = self.counts.get("hd_confirm_crawls", 0) + 1
            self.pump(r)
            return
        if hc["stage"] == "measure":
            c1 = [q for q in r.cam_hist if q[0] >= hc["t_end"] - 0.05]
            if len(c1) < 2 and self.t - hc["t_end"] < 3.0:
                r.busy_until = self.t + 0.2
                return
            if not c1:
                hc["stage"] = None
                hc["tries"] += 1
                return
            ex = sum(q[1] for q in c1) / len(c1)
            ey = sum(q[2] for q in c1) / len(c1)
            moved = math.hypot(ex - hc["c0"][0], ey - hc["c0"][1])
            hc["tries"] += 1
            if moved < 0.5 * hc["L"]:
                self.event(r, "heading not confirmed", "the camera saw it move only %.1f cm of %.0f" % (moved, hc["L"]))
                if hc["tries"] >= 3:
                    r.hd_confirm = None
                else:
                    hc["stage"] = None
                return
            cam_h = math.degrees(math.atan2(ey - hc["c0"][1], ex - hc["c0"][0]))
            err = wrap180(cam_h - hc["h_own"])
            self.log("heading", dict(t=round(self.t, 2), robot=r.name, kind="confirm crawl", err_deg=round(err, 1),
                                     moved_cm=round(moved, 1), in_a_row=hc["tries"]))
            if abs(err) <= HD_CONFIRM_DEG:
                r.hd_confirm = None
                self.counts["hd_confirmed"] = self.counts.get("hd_confirmed", 0) + 1
                self.event(r, "heading confirmed", "a %.0f cm crawl: the camera saw it go %.0f deg, its own heading "
                           "%.0f (%.1f off)" % (moved, cam_h, hc["h_own"], err))
                return
            h = wrap180(self.heading(r) + err)
            self.out(r, "fpose %.2f %.2f %.2f 1.0 3.0" % (ex, ey, h))
            r.fpose_t = self.t
            self.counts["hd_confirm_set"] = self.counts.get("hd_confirm_set", 0) + 1
            self.event(r, "heading set", "a %.0f cm crawl: the camera saw it go %.0f deg, its own heading said %.0f: "
                       "its heading set from the camera and checked again" % (moved, cam_h, hc["h_own"]))
            if hc["tries"] >= 3:
                r.hd_confirm = None
            else:
                hc["stage"] = None
                hc["t_idle"] = self.t + 0.3

    def hdconfirm_over(self, r, mv, state):
        r.move = None
        r.mode = "idle"
        for st in r.steps.values():
            if st["status"] in ("queued_local", "sent", "acked", "started"):
                st["status"] = "dropped"
        hc = r.__dict__.get("hd_confirm")
        if hc is None:
            return
        hc["stage"] = "measure"
        hc["t_end"] = self.t
        r.busy_until = self.t + 0.2

    # -- moves ---------------------------------------------------------------------------------------------------
    def others(self, r):
        return [o for o in self.R.values() if o is not r and o.traj is not None and o.mode != "gone"]

    def unc(self, r):
        """(11-05h) Its pose uncertainty now (cm): the camera's error, the bubble when lost, the camera's staleness
        and the clock's doubt (the measured delay's upper end) at its planned speed."""
        import observe as OB
        age = (self.t - (r.cam_arr if r.cam_arr is not None else r.cam[0])) if (r.cam is not None and r.lost is None) \
            else 0.0
        if STAND_TRUST and STAND_UNC and r.lost is None and self.standing(r):
            # (e16, 9b; home 11 Nov: Center "cannot tell whether it is clear of the edge" 22 cm from it) a robot that
            # has not moved since its last fix cannot have gone anywhere: the fix's age adds nothing while it stands
            # (e13b kept the fix fresh but this term still grew at 10 cm/s: 2 s without a picture = 20 cm of doubt)
            age = 0.0
        u = OB.uncertainty_cm(PLAN_M0, r.bubble if r.lost else 0.0, age, 10.0, 0.0)
        if self.ekf_live(r):
            u = r.fp.unc_cm(self.t)                 # (e09, B2) the filter's: staleness and blind driving inside it
        pf = r.__dict__.get("pfuse")
        # (e08, A1) its own pose vs the camera: the disagreement widens the uncertainty AT ONCE (no 2 s wait)
        return u + (pf.unc_extra() if (pf is not None and r.lost is None) else 0.0)

    # -- (e08, A2) the derived edge rule on the whole swept path ----------------------------------------------------
    def need_cm(self, r, speed=None, stops_here=False, unc=None):
        """The derived edge clearance for r now (safety_rules.clearance_cm): half diagonal + its uncertainty + its stop
        distance where it stops + the camera's edge bias for this tracking."""
        return SR.clearance_cm(self.unc(r) if unc is None else unc, speed, self.tracking, stops_here,
                               self.lead_tab.get(r.name))

    def edge_ok(self, r, tr, speed=None, away=False, stops=True):
        """(e08, A2) Every sample of the move (turns on the spot: the body circle covers any heading; a curve's loop disc:
        its rim) keeps the derived clearance from every edge, the uncertainty growing with the plan's own execution
        margin (and the bubble when lost); the stop distance at its end (stops=False: a curve piece runs on into its
        loop disc, which is checked with the stop distance on its rim). A robot already inside the clearance (or away)
        may only take a move whose margin never gets worse and ends better. Fixed rule: the old checks."""
        if self.edge_rule == "fixed":
            if not self.invariant_ok(r, tr):
                return False
            if not getattr(tr, "loop_c", None) is not None and edge_dist(self.rect, *tr.end) < EDGE_CC + (
                    r.bubble if r.lost else 0.0):
                return False
            return True
        u0 = self.unc(r)
        p = self.pos(r)
        loop = getattr(tr, "loop_c", None) is not None
        xs, ys, ms = (tr.x[:-1], tr.y[:-1], tr.m[:-1]) if loop else (tr.x, tr.y, tr.m)
        grow = ms - (ms[0] if len(ms) else PLAN_M0)
        mar = []
        if p is not None:
            mar.append(edge_dist(self.rect, *p) - self.need_cm(r, None, False, u0))
        n = len(xs)
        for k in range(n):
            need = self.need_cm(r, speed, False, u0 + max(0.0, float(grow[k])))
            mar.append(edge_dist(self.rect, xs[k], ys[k]) - need)
        if stops and not loop and n:
            # its stopping distance: the overrun carries on along the move's last direction (the path is extended by
            # it, so an inward move's overrun counts as inward and an outward one's as outward)
            px, py = (xs[-2], ys[-2]) if n >= 2 else (p if p is not None else (xs[-1], ys[-1]))
            dx, dy = xs[-1] - px, ys[-1] - py
            L = math.hypot(dx, dy)
            if L > 1e-6:
                sc = SR.stop_cm(speed, self.lead_tab.get(r.name)) / L
                ex, ey = xs[-1] + dx * sc, ys[-1] + dy * sc
                mar.append(edge_dist(self.rect, ex, ey) - self.need_cm(r, speed, False, u0 + max(0.0, float(grow[-1]))))
        if loop:
            c = tr.loop_c        # the loop disc's rim: where it may coast and brake, any direction: the stop added
            need = self.need_cm(r, speed, True, u0 + max(0.0, float(tr.m[-1] - LOOP_R - (ms[0] if len(ms) else 0))))
            mar.append(edge_dist(self.rect, *c) - LOOP_R - need)
        m0 = mar[0]
        if not away and m0 >= 0:
            ok = min(mar) >= 0
        else:
            # the shrink rule (observe.path_ok's, with the derived clearance): never worse, and ends better
            ok = all(m >= min(0.0, m0) - 0.5 for m in mar) and (mar[-1] >= 0 or mar[-1] > m0 + 0.5)
        if not ok:
            self.counts["edge_refused"] = self.counts.get("edge_refused", 0) + 1
            return False
        # the keep-outs and the possible-locations circle (observe.path_ok) stay as they were
        if not self.invariant_ok(r, tr, edge=False):
            return False
        return True

    def turn_need(self, r):
        """A turn on the spot: its corners sweep the body circle: the derived clearance without a stop (fixed: 9.2)."""
        if self.edge_rule == "fixed":
            return TURN_EDGE_CC + (r.bubble if r.lost else 0.0)
        return self.need_cm(r, None, False)

    def end_need(self, r, speed=None):
        """Where a move ends (a park spot, the reflex's new end): the derived clearance with the stop (fixed: 16)."""
        if self.edge_rule == "fixed":
            return EDGE_CC + (r.bubble if r.lost else 0.0)
        return self.need_cm(r, speed, True)

    def invariant_ok(self, r, tr, edge=True):
        """(11-05h, Nick 21:14, hard invariant) The possible-locations circle (half diagonal + its uncertainty, grown
        per cm when lost) never overlaps the board's edge (every side a drop, home too) or a keep-out anywhere along
        the move's samples, the turn's sweep included; a robot already overlapping may only shrink it."""
        import observe as OB
        pts = [(float(x), float(y)) for x, y in zip(tr.x, tr.y)]
        p = self.pos(r)
        if p is not None:
            pts = [tuple(p)] + pts
        kos = [(q["x"], q["y"], q["r"]) for q in self.keepouts if q.get("owner") != r.name]
        rect = self.rect if edge else (-1e6, -1e6, 1e6, 1e6)      # (e08) the edges: edge_ok's derived rule
        ok, why = OB.path_ok(pts, self.unc(r), rect, kos, BLIND_PER_CM if r.lost is not None else 0.0)
        if not ok:
            self.counts["invariant_refused"] = self.counts.get("invariant_refused", 0) + 1
        return ok

    def feasible(self, r, tr, edge_extra=0.0, gap_relax=0.0, ignore=(), away=False):
        """away: a robot that already stands too close may still take a move that never brings it closer (each
        sample at least its start's clearance - 0.5 cm) and ends clear (the escape and yield rungs)."""
        if self.edge_rule == "fixed":
            if not self.invariant_ok(r, tr):
                return False
            e = tr.end
            if edge_dist(self.rect, e[0], e[1]) < EDGE_CC + edge_extra + (r.bubble if r.lost else 0.0):
                return False
        else:
            spd = getattr(tr, "v", None)
            if not self.edge_ok(r, tr, spd, away=away):
                return False
            e = tr.end
            if edge_extra and edge_dist(self.rect, e[0], e[1]) < self.end_need(r) + edge_extra:
                return False
        if not self.keepout_ok(r, tr):
            return False
        for o in self.others(r):
            if o.name in ignore:
                need = REFLEX_CC
                ob = Traj.stand(self.t, *self.pos(o), m=self.margin0(o))
            else:
                need = PLAN_CC - gap_relax
                ob = o.traj
            if self.pinned(o) and self.pos(o) is not None:
                # (e16, 8b) held or stuck at an edge: a fixed obstacle where it stands, with its doubt, whatever its own
                # reservation says (home 08:48: Center pushed a held Right off the board)
                ob = Traj.stand(self.t, *self.pos(o), m=self.margin0(o) + self.pinned_extra(o))
                need = max(need, REFLEX_CC)
            if not away:
                if clearance(tr, ob, now=self.t) < need:
                    return False
                continue
            prof = clearance_profile(tr, ob, now=self.t)
            if prof[-1] < need:
                return False
            if (prof < np.minimum(need, prof[0] - 0.5)).any():
                return False
        return True

    def keepout_ok(self, r, tr):
        """(11-05c) Every sample of the move keeps its circle KEEP_GAP from every discovered keep-out; a robot that
        already stands too close may take a move that never brings it closer and ends clear (backing away)."""
        kos = [q for q in self.keepouts if q.get("owner") != r.name]     # (11-05h) not its own tilt spot
        if not kos:
            return True
        k = np.array([(q["x"], q["y"], q["r"]) for q in kos])
        d = np.hypot(tr.x[:, None] - k[None, :, 0], tr.y[:, None] - k[None, :, 1]) - k[None, :, 2] - tr.m[:, None] \
            - HALF - KEEP_GAP + PLAN_M0
        dm = d.min(axis=1)
        if (dm >= 0).all():
            return True
        return bool(dm[-1] >= 0 and (dm >= np.minimum(0.0, dm[0] - 0.5)).all())

    def felt(self, r, info):
        """(11-05c) Its own front sensor (or edge guard) stopped a step: where the thing it felt is. Next to another
        robot's known spot it is that robot (no keep-out); otherwise a keep-out for the rest of the run (discovered:
        a wall, an object, a robot the camera does not see), cleared again only if a robot's camera track later
        drives through it. The next move backs away."""
        p, h = self.pos(r), self.heading(r)
        if p is None:
            return
        import re as _re
        m = _re.search(r"obstacle (-?[0-9.]+) cm", info)
        d = max(0.0, float(m.group(1))) if m else 1.0
        u = (math.cos(math.radians(h)), math.sin(math.radians(h)))
        f = (p[0] + u[0] * (BODY / 2 + d), p[1] + u[1] * (BODY / 2 + d))
        r.back_off = dict(t=self.t, h=h, edge=("edge" in info or "tilted" in info) and not m)   # (e16) edge stops
        self.counts["front_stops"] += 1
        who = [o.name for o in self.R.values() if o is not r and self.pos(o) is not None and
               math.dist(self.pos(o), f) < HALF + 3.0]
        src = "edge guard" if "edge" in info and not m else "front sensor"
        if "tilted" in info:
            src = "tilt"
        if src == "edge guard" and " line" in info:
            # (e08, A5; Nick 6 Nov: surface marks logged, nothing else) a line-sensor trip the bottom zone does not
            # confirm, with the camera putting the robot well inside the board, is a mark on the surface: logged, no
            # keep-out (observe.edge_trip_kind, the rule run_plan uses)
            import observe as OB
            cam_e = edge_dist(self.rect, r.cam[1], r.cam[2]) if (r.cam is not None and self.fresh(r)) else None
            # (e10, run 2 incident b) the camera's distance less the fused pose's uncertainty: 9.4 cm in with 6 cm of
            # doubt is not "well inside" (MIRLT_MARK_UNC=0: the camera's distance alone, as before)
            u_m = self.unc(r) if (self.mark_unc and cam_e is not None) else 0.0
            kind = OB.edge_trip_kind(True, False, cam_e - u_m if cam_e is not None else None,
                                     0.0 if cam_e is not None else None)
            if self.mark_unc and kind == "edge" and cam_e is not None and cam_e > OB.SURFACE_MARK_CAM_CM:
                self.counts["marks_deferred"] = self.counts.get("marks_deferred", 0) + 1
                self.event(r, "edge, not a mark", "its line sensors saw dark %.0f cm inside the board by the camera, "
                           "but the position is only sure to %.0f cm: treated as the edge" % (cam_e, u_m))
            self.log("surface_marks", dict(t=round(self.t, 2), robot=r.name, x=round(f[0], 1), y=round(f[1], 1),
                                           cam_edge_cm=round(cam_e, 1) if cam_e is not None else "", kind=kind,
                                           info=info, unc_cm=round(u_m, 2)))
            if kind == "surface_mark":
                self.counts["surface_marks"] = self.counts.get("surface_marks", 0) + 1
                self.event(r, "surface mark", "its line sensors saw dark %.0f cm inside the board by the camera, the "
                           "bottom zone saw floor: a mark on the surface, logged (nothing mapped)" % cam_e)
                return
        if self.ws is not None and src == "edge guard":
            # (e10, C2) the proven workspace: a drop felt there, never proven again this run
            reach = BOTTOM_REACH_CM if "bottom" in info else LINE_REACH_CM
            k = self.ws.drop_at(p[0], p[1], h, reach)
            if k:
                self.event(r, "drop felt", "its %s felt a drop %.0f cm ahead at %.1f, %.1f: %d cells of the workspace "
                           "marked as a drop" % ("bottom zone" if "bottom" in info else "line sensors", reach, p[0],
                                                 p[1], k))
        if who:
            self.event(r, "felt a robot", "%s stopped it %.1f cm ahead at %.1f, %.1f: that is %s (no keep-out)" % (
                src, d, f[0], f[1], who[0]))
            return
        self.wall_check(r, f, src, h)
        # (e08, A6, one rule with run_plan: safety_rules.keepout_confirmed) a second felt point near an earlier one
        # (still on the map or faded) confirms it; a real drop, edge or tilt is confirmed at once; a single sensor-only
        # stop is unconfirmed and fades after safety_rules.UNCONFIRMED_S
        hist = self.__dict__.setdefault("felt_hist", [])
        before = any(math.dist((hx, hy), f) < SR.CONFIRM_NEAR_CM for hx, hy in hist)
        hist.append((f[0], f[1]))
        del hist[:-400]
        conf = SR.keepout_confirmed(src, felt_before=before)
        for q in self.keepouts:
            if math.dist((q["x"], q["y"]), f) < 1.0 or (conf and not q.get("confirmed", True) and
                                                        math.dist((q["x"], q["y"]), f) < SR.CONFIRM_NEAR_CM):
                if conf and not q.get("confirmed", True):
                    q["confirmed"] = True
                    self.log("keepouts", dict(q, what="confirmed (felt again)"))
                if math.dist((q["x"], q["y"]), f) < 1.0:
                    return
        q = dict(x=round(f[0], 1), y=round(f[1], 1), r=KEEP_R, t=round(self.t, 2), robot=r.name, src=src,
                 edge_cm=round(edge_dist(self.rect, *f), 1), d_cm=d, confirmed=conf)
        self.keepouts.append(q)
        self.counts["keepouts"] += 1
        self.log("keepouts", dict(q, what="found"))
        self.event(r, "keep-out", "%s felt something %.1f cm ahead at %.1f, %.1f (%.1f cm inside the board's edge): %s"
                   % (src, d, f[0], f[1], q["edge_cm"], "kept out of for the rest of the run (confirmed)" if conf or
                      self.keepout_rule == "permanent" else "unconfirmed: fades in %.0f s unless felt again"
                      % SR.UNCONFIRMED_S))

    def wall_check(self, r, f, src, h=None):
        """(11-05c) Discover, don't pre-map: a thing felt inside the edge band (closer to a side than a planned body's
        face can be, EDGE_CC - HALF) is that side's real wall or edge: the side moves in to it for the rest of the run
        (never out; at most WALL_MAX_IN, beyond that it is an object, a keep-out only)."""
        x0, y0, x1, y1 = self.rect if self.ws is None else self.ws.board
        sides = [(f[0] - x0, 0), (f[1] - y0, 1), (x1 - f[0], 2), (y1 - f[1], 3)]
        d, k = min(sides)
        if d < WALL_MIN_IN or d > EDGE_CC - HALF or d > WALL_MAX_IN:
            return
        if h is not None:                    # only when it faced that side (a glancing touch says little)
            normal = (180.0, -90.0, 0.0, 90.0)[k]
            if abs(wrap180(h - normal)) > 35.0:
                return
        # two felt points on the same side at about the same depth (+-3 cm), 15 cm or more apart along it
        pend = self.__dict__.setdefault("wall_pend", {}).setdefault(k, [])
        twin = [q for q in pend if abs(q[0] - d) <= 3.0 and abs(q[1] - f[1 - k % 2]) >= 15.0]
        pend.append((d, f[1 - k % 2]))
        if not twin:
            self.event(r, "wall?", "its %s felt something %.1f cm inside the %s side: one more there and that side "
                       "moves in" % (src, d, ("x_min", "y_min", "x_max", "y_max")[k]))
            return
        d = min(d, min(q[0] for q in twin))
        rect = list(self.rect if self.ws is None else self.ws.board)
        rect[k] = rect[k] + d if k < 2 else rect[k] - d
        if self.ws is not None:
            # (e10, C2) the proven workspace: every cell past the felt side is a drop from now on
            self.ws.cut_side(k, rect[k])
            self.ws.board = tuple(rect)
        else:
            self.rect = tuple(rect)
        self.log("keepouts", dict(x=round(f[0], 1), y=round(f[1], 1), r=0.0, t=round(self.t, 2), robot=r.name, src=src,
                                  edge_cm=round(d, 1), d_cm="", what="wall: the %s side moved in %.1f cm" % (
                                      ("x_min", "y_min", "x_max", "y_max")[k], d)))
        self.event(r, "wall found", "its %s felt the board's %s side %.1f cm inside where the calibration has it: "
                   "board now x %.1f-%.1f, y %.1f-%.1f cm" % (src, ("x_min", "y_min", "x_max", "y_max")[k], d,
                                                             *[rect[i] for i in (0, 2, 1, 3)]))

    def clear_keepouts(self):
        """A keep-out a robot's verified camera track drives through was not a fixed thing (a robot the camera did not
        see then, a hand): dropped."""
        for q in list(self.keepouts):
            # (e08, A6) unconfirmed (one sensor-only stop) keep-outs fade after safety_rules.UNCONFIRMED_S
            if not SR.keepout_alive(q, self.t, self.keepout_rule):
                self.keepouts.remove(q)
                self.counts["keepouts_faded"] = self.counts.get("keepouts_faded", 0) + 1
                self.log("keepouts", dict(q, what="faded (unconfirmed for %.0f s)" % SR.UNCONFIRMED_S))
        for r in self.R.values():
            if r.lost is not None or not self.fresh(r) or r.mode in ("out", "gone"):
                continue
            p = self.pos(r)
            for q in list(self.keepouts):
                if q.get("owner"):
                    # (11-05h) a tilted robot's own keep-out: dropped once it has left it (backed off, level again)
                    if q["owner"] == r.name and math.dist(p, (q["x"], q["y"])) > q["r"] + 5.0 and \
                            not getattr(r, "tilted", False):
                        self.keepouts.remove(q)
                        self.log("keepouts", dict(q, what="%s left its tilt spot" % r.name))
                    continue
                if math.dist(p, (q["x"], q["y"])) < HALF - 1.0 and self.t - q["t"] > 2.0:
                    self.keepouts.remove(q)
                    self.log("keepouts", dict(q, what="cleared by %s's camera track" % r.name))
                    self.event(r, "keep-out cleared", "its camera track went through the keep-out at %.1f, %.1f"
                               % (q["x"], q["y"]))

    def track_backout(self, r):
        """(e16, item 7; Nick 11 Nov 08:45: "it can still back up but it's not, it's just still") A robot at an edge
        came in over ground it has just driven: it reverses along its own recorded track (observe.track_back_target)
        to the first point of it where the derived edge rule is met (its clearance with its error sum and the stop
        where it ends: end_need), as one straight chord that stays on the track. The reverse is checked against the
        others' reservations (with a held robot's doubt, 8b) and the keep-outs. A curved track gives a shorter chord
        and it goes on from its end (the trail keeps only what is still behind it). None: no track behind it, or it is
        blocked: the caller holds and asks (never gives up)."""
        import observe as OB
        p, h = self.pos(r), self.heading(r)
        tr_ = getattr(r, "trail", None)
        if p is None or tr_ is None or len(tr_.pts) < 2:
            self.event(r, "no track to back out on", "its own track behind it is not recorded: it holds")
            return None
        need = self.end_need(r, BACK_V)
        tgt = OB.track_back_target(tr_.pts, p, h, lambda q: edge_dist(self.rect, *q) >= need,
                                   tol_cm=OB.TRAIL_TOL_CM + (r.bubble if r.lost else 0.0))
        if tgt is None or tgt["L"] < 1.0:
            self.counts["track_back_none"] = self.counts.get("track_back_none", 0) + 1
            return None
        L = tgt["L"]
        tr = Traj.move(self.t + 0.2, p, wrap180(h + 180.0), 0.0, L, BACK_V, m0=self.margin0(r))
        tr.track_back = True
        if not self.keepout_ok(r, tr):
            self.event(r, "track back blocked", "a keep-out lies on its own track behind it: it holds")
            return None
        for o in self.others(r):
            ob = o.traj
            if self.pinned(o):
                ob = Traj.stand(self.t, *self.pos(o), m=self.margin0(o) + self.pinned_extra(o))
            if clearance(tr, ob, now=self.t) < REFLEX_CC:
                self.counts["track_back_blocked"] = self.counts.get("track_back_blocked", 0) + 1
                self.event(r, "track back blocked", "%s is on or near its own track behind it: it holds" % o.name)
                return None
        k = tgt["k"]
        tr_.pts = tr_.pts[:k + 1]          # what is still behind it once it has reversed there
        self.counts["track_backs"] = self.counts.get("track_backs", 0) + 1
        self.log("track_back", dict(t=round(self.t, 2), robot=r.name, x=round(p[0], 1), y=round(p[1], 1),
                                    L_cm=L, need_cm=round(need, 2), edge_cm=round(edge_dist(self.rect, *p), 2),
                                    cleared=int(tgt["cleared"])))
        self.event(r, "backs out along its track", "%s (the edge rule needs %.1f cm from the edge; it is %.1f cm)" % (
            tgt["words"], need, edge_dist(self.rect, *p)))
        self.fallback(r, 0, "reverse", "an edge stop: back along its own track (proven table), then re-plan")
        if not tgt["cleared"]:
            r.back_off = dict(t=self.t, h=h, edge=True)        # not clear yet: the next chord after this one
        return "reverse", None, 0.0, L, BACK_V, tr

    # -- (e16, 8c) pushes and collisions ---------------------------------------------------------------------------
    def contact(self, r, why, src):
        """A contact: logged (contact.csv, event), and every robot near r (within a reflex gap plus what it covers at top
        speed in the reflex's reaction time, plus a pinned robot's doubt) is stopped and backs away; r too if it drives."""
        self.counts["contacts"] = self.counts.get("contacts", 0) + 1
        p = self.pos(r)
        reach = PLAN_CC + V_MAX * HARD_T_S
        near = [o for o in self.R.values() if o is not r and o.mode not in ("out", "gone") and p is not None and
                self.pos(o) is not None and math.dist(p, self.pos(o)) <= reach + self.pinned_extra(o)]
        stopped = []
        for o in near + [r]:
            if o.move is not None:
                self.halt(o, "contact at %s: %s" % (r.name, why))
                self.end_move(o)
                o.back_off = dict(t=self.t, h=self.heading(o), edge=False)
                o.busy_until = self.t + 0.5
                stopped.append(o.name)
        self.log("contact", dict(t=round(self.t, 2), robot=r.name, src=src, why=why,
                                 x=round(p[0], 1) if p else "", y=round(p[1], 1) if p else "",
                                 near=";".join(o.name for o in near), stopped=";".join(stopped)))
        self.event(r, "contact", "%s (%s); stopped: %s" % (why, src, ", ".join(stopped) or "nobody was moving"))

    def contact_watch(self, r):
        """(e16, 8c) Every half second. Standing (not commanded): its camera spot moving past its own measured noise,
        sustained (observe.moved_in_evidence, the same test as a hand's move) is a push. Driving: its camera progress
        over PROGRESS_WIN_S under PROGRESS_MIN of its own wheels' is pushing or blocked: it stops and backs off."""
        if r.mode in ("out", "gone", "start") or r.cam is None or r.lost is not None or not self.fresh(r):
            return
        import observe as OB
        cw = r.__dict__.setdefault("cwatch", dict(fx=[], t_idle=None))
        if r.mode == "idle" and r.move is None:
            if cw["t_idle"] is None:
                cw.update(t_idle=self.t, fx=[])
            if self.t - cw["t_idle"] < 1.0:
                return                                 # settling after its last move
            fx = cw["fx"]
            if not fx or fx[-1][2] != r.cam[0]:
                fx.append((r.cam[1], r.cam[2], r.cam[0]))
                del fx[:-60]
            if len(fx) >= 8:
                ok, ev = OB.moved_in_evidence([f[:2] for f in fx[:-3]][-30:], [f[:2] for f in fx], k_sigma=CONTACT_K)
                if ok:
                    cw.update(t_idle=self.t, fx=[])
                    self.contact(r, "moved while standing (nothing commanded): " + ev, "camera")
            return
        cw["t_idle"] = None
        mv = r.move
        if mv is None or mv.get("phase") != "drive" or len(r.cam_hist) < 3 or len(r.own_hist) < 3 or \
                mv.get("t_drive") is None or self.t - mv["t_drive"] < PROGRESS_WIN_S:
            cw["slow"] = 0
            return
        c1, o1 = r.cam_hist[-1], r.own_hist[-1]
        ch = [q for q in r.cam_hist if q[0] >= c1[0] - PROGRESS_WIN_S]
        oh = [q for q in r.own_hist if q[0] >= o1[0] - PROGRESS_WIN_S]
        if len(ch) < 3 or len(oh) < 3 or ch[-1][0] - ch[0][0] < 0.7 * PROGRESS_WIN_S:
            return
        wheel = math.hypot(oh[-1][1] - oh[0][1], oh[-1][2] - oh[0][2])
        cam = math.hypot(ch[-1][1] - ch[0][1], ch[-1][2] - ch[0][2])
        if wheel < 4.0 * math.sqrt(2.0) * 1.0:        # its wheels' travel must clear the camera's noise (1 cm sigma)
            return
        cw["slow"] = cw.get("slow", 0) + 1 if cam < PROGRESS_MIN * wheel else 0
        if cw["slow"] >= PROGRESS_N:
            cw["slow"] = 0
            self.contact(r, "its camera progress %.1f cm against its wheels' %.1f cm in %.1f s: pushing or blocked" % (
                cam, wheel, PROGRESS_WIN_S), "progress")

    def on_raw_motion(self, name, t, ax, ay, lifted=None):
        """(e16, 8c, 8a) The robot's accelerometer (RAW ax, ay in g) and its own lift flag: a jolt while standing (a
        robot or a hand hit it) is a contact; a lift is the evidence that a person moved it (moved in)."""
        r = self.R.get(name)
        if r is None or ax is None or ay is None:
            return
        if lifted:
            r.lifted_t = self.t
        if not CONTACT_WATCH:
            return
        hist = r.__dict__.setdefault("acc_hist", [])
        if r.mode != "idle" or r.move is not None:
            del hist[:]
            return
        a = math.hypot(ax, ay)
        hist.append(a)
        del hist[:-200]
        if len(hist) < 30:
            return
        base = sorted(hist[:-1])
        med = base[len(base) // 2]
        dev = sorted(abs(v - med) for v in base)
        sig = 1.4826 * dev[len(dev) // 2]
        if abs(a - med) > max(IMPACT_K * sig, IMPACT_FLOOR_G):
            del hist[:]
            self.contact(r, "a jolt of %.2f g while standing (its noise %.3f g)" % (abs(a - med), sig), "accelerometer")

    def pinned(self, r):
        """(e16, 8b) Held or stuck at an edge (waiting for a hand, unsure and holding, tilted), or rejoined and not yet
        driven away: every other robot plans round it as a fixed obstacle, with its doubt."""
        if not PINNED_OBSTACLE or r.mode in ("gone",):
            return False
        if r.name in (self.__dict__.get("need_hand") or {}) or self.edge_stuck_why(r) is not None:
            return True
        if getattr(r, "tilted", False) or getattr(r, "held", None):
            return True
        rj = r.__dict__.get("rejoined_at")
        if rj is not None:
            p = self.pos(r)
            if p is not None and math.dist(p, rj) <= HALF:
                return True
            r.rejoined_at = None
        return False

    def pinned_extra(self, r):
        """(e16, 8b) How much wider a pinned robot is kept: its own uncertainty beyond the plan's margin, and at least
        the distance between where its own pose and the camera put it (home 08:48: 3.4-6.8 cm), so it is kept clear
        wherever of the two it really is."""
        if not self.pinned(r):
            return 0.0
        d = 0.0
        if r.own is not None and r.cam is not None:
            d = math.hypot(r.own[1] - r.cam[1], r.own[2] - r.cam[2])
        return max(0.0, self.unc(r) - PLAN_M0, d)

    def back_move(self, r, Ls=BACK_CM, why="its own front stop", inward=False):
        """Straight backwards (firmware rev drive: no front check; no rear sensor, so short and slow) along its
        heading: the longest of Ls that keeps the others' reservations and the keep-outs. inward: only if it ends
        further from the edge than it starts."""
        p, h = self.pos(r), self.heading(r)
        import observe as OB
        # (11-05h, 4'; Nick 21:11) a reverse only over space shown safe by evidence that fits its uncertainty: its own
        # breadcrumb trail (no camera needed), or a fresh camera fix showing room behind; else none (the ladder's
        # turn on the spot then forward is the "look before backing")
        e0 = edge_dist(self.rect, *p)
        others = [self.pos(o) for o in self.others(r) if self.pos(o) is not None]
        kos = [(q["x"], q["y"], q["r"]) for q in self.keepouts if q.get("owner") != r.name]
        fresh = r.cam is not None and self.fresh(r) and r.lost is None
        fm = getattr(r, "fmem", None)
        for L in Ls:
            if fm is not None and fm.same("back off" if not inward else "reverse", -1, L, p[0], p[1]):
                continue                               # (3b) failed here before: not repeated unchanged
            how, L_ok, why_ = OB.back_out_plan(L, trail=getattr(r, "trail", None), bubble_cm=r.bubble if r.lost
                                               else 0.0, cam=(p[0], p[1], h) if fresh else None,
                                               cam_age_s=0.0 if fresh else None, rect=self.rect, others=others,
                                               keepouts=kos)
            if how not in ("reverse_trail", "reverse_camera") or L_ok < L - 0.5:
                self.counts["reverse_refused"] = self.counts.get("reverse_refused", 0) + 1
                continue
            tr = Traj.move(self.t + 0.2, p, wrap180(h + 180.0), 0.0, L, BACK_V, m0=self.margin0(r))
            e1 = edge_dist(self.rect, *tr.end)
            # backwards is blind: away from the edges by the turn margin, or (inward) at least further in than now.
            # (e08, A2) not inward: the derived clearance with its stop where it ends
            far = TURN_EDGE_CC + (r.bubble if r.lost else 0.0) if self.edge_rule == "fixed" else \
                self.end_need(r, BACK_V)
            if e1 < (HALF + 1.0 + (r.bubble if r.lost else 0.0) if inward else far) or \
                    (inward and e1 < e0 + (INWARD_GAIN_CM if Ls == CORNER_BACK_CM else 1.0)):
                continue
            if not self.keepout_ok(r, tr) or not self.invariant_ok(r, tr):
                continue
            if self.edge_rule != "fixed" and not self.edge_ok(r, tr, BACK_V, away=inward):
                continue                     # (e08, A2) blind backing: the derived rule on its whole path too
            if all(clearance_profile(tr, o.traj, now=self.t)[-1] >= REFLEX_CC for o in self.others(r)):
                return ("back off" if not inward else "reverse"), None, 0.0, L, BACK_V, tr
        return None

    def make(self, r, turn, L, v, t0=None):
        p, h = self.pos(r), self.heading(r)
        grow = BLIND_PER_CM if r.lost is not None else 0.0
        return Traj.move(self.t + 0.2 if t0 is None else t0, p, h, turn, L, v, m0=self.margin0(r), bubble_grow=grow)

    def can_turn_here(self, r):
        p = self.pos(r)
        return edge_dist(self.rect, *p) >= self.turn_need(r)

    def try_probe(self, r):
        """(e10, C2, the probe-ahead setting) A robot standing near ground nobody has proven (the camera cannot see it,
        a dark patch, past the typed board) drives slowly straight into it from proven ground: the planner lets the move
        through a temporary corridor (PROBE cells) which the robot's own bottom zone and line sensors guard; what its
        wheels roll over is proven (felt), a drop it feels is marked. Returns a stop-turn move or None."""
        ws = self.ws
        if ws is None or not self.probe_ahead or r.lost is not None or getattr(r, "tilted", False) or \
                self.t - r.__dict__.get("probe_t", -1e9) < PROBE_EVERY_S:
            return None
        p, h = self.pos(r), self.heading(r)
        if p is None or h is None:
            return None
        r.probe_t = self.t
        fr = ws.frontier(p[0], p[1], PROBE_REACH_CM)
        if not fr:
            return None
        turn_ok = self.can_turn_here(r)
        step = max(1, len(fr) // 16)
        for d, ux, uy, di, dj in fr[::step][:16]:
            dx, dy = -float(di), -float(dj)
            T = (ux + dx * PROBE_IN_CM, uy + dy * PROBE_IN_CM)
            L = math.dist(p, T)
            if L < 6.0 or L > PROBE_MAX_L:
                continue
            hd = math.degrees(math.atan2(T[1] - p[1], T[0] - p[0]))
            if abs(wrap180(hd - math.degrees(math.atan2(dy, dx)))) > 35.0:
                continue                         # it must meet the unknown ground nearly head-on (its sensors look ahead)
            turn = wrap180(hd - h)
            if abs(turn) >= 3.0 and not turn_ok:
                continue
            # the corridor: from 15 cm before the unknown ground to the probe's end, as wide as the edge rule asks along
            # a path this long (its doubt grows with the length: PLAN_M_MAX) plus its stop
            cells = ws.provisional(ux - dx * 15.0, uy - dy * 15.0, T[0], T[1],
                                   self.end_need(r, PROBE_V) + PLAN_M_MAX + 2.0)
            tr = self.make(r, turn, L, PROBE_V)
            ok = self.feasible(r, tr)
            if not ok:
                self.counts["probe_refused"] = self.counts.get("probe_refused", 0) + 1
            if ok:
                r.probe_cells = cells
                self.counts["probes"] = self.counts.get("probes", 0) + 1
                self.fallback(r, 0, "probe", "probes %.0f cm into ground nobody has proven (%.0f, %.0f), at %.0f cm/s"
                              % (PROBE_IN_CM, T[0], T[1], PROBE_V))
                return "probe", None, turn, L, PROBE_V, tr
            ws.clear_provisional(cells)
        return None

    def probe_over(self, r):
        """(e10, C2) A probe that is over, by any path (done, an edge felt, halted, lost from view): what its wheels
        rolled over is proven, the rest of its corridor unknown again, and it comes back the way it went."""
        pa = r.__dict__.get("probe_active")
        if pa is None or (r.mode == "move" and r.move is not None and r.move.get("kind") == "probe"):
            return
        r.probe_active = None
        self.felt_mark(r)
        if self.ws is not None:
            self.ws.clear_provisional(pa["cells"])
        r.probe_ret = dict(p0=pa["p0"])

    def probe_return(self, r):
        """(e10, C2) After a probe: straight back (reversing) along the strip it just proved, towards where it started;
        the strip is felt table and the start was proven. Only the keep-outs and the others' plans are checked; when
        another robot's plan holds the far end, a shorter way back (at least to seen ground); tried again while it is
        still out on unproven ground."""
        ret = r.__dict__.get("probe_ret")
        self.counts["probe_return_tries"] = self.counts.get("probe_return_tries", 0) + 1
        p, h = self.pos(r), self.heading(r)
        if ret is None or ret.get("p0") is None or p is None or h is None:
            r.probe_ret = None
            return None
        Lf = math.dist(p, ret["p0"])
        ret["tries"] = ret.get("tries", 0) + 1
        if Lf < 2.0 or (ret["tries"] > 20 and self.ws is not None and self.ws.state(*p) == 1):
            r.probe_ret = None                       # back on seen ground: the usual rules from here
            return None
        for L in sorted({Lf, 0.75 * Lf, 0.5 * Lf, min(Lf, 20.0)}, reverse=True):
            tr = Traj.move(self.t + 0.2, p, wrap180(h + 180.0), 0.0, L, BACK_V, m0=self.margin0(r))
            if not self.keepout_ok(r, tr):
                continue
            if any(clearance_profile(tr, o.traj, now=self.t)[-1] < REFLEX_CC for o in self.others(r)):
                continue
            r.probe_ret = None if L >= Lf - 0.5 else dict(p0=ret["p0"])
            self.fallback(r, 0, "probe return", "back %.0f cm along the strip it just proved" % L)
            return "probe return", None, 0.0, L, BACK_V, tr
        return None

    def felt_mark(self, r):
        """(e10, C2) The ground under a robot's wheels since the last mark is table (it did not hang or fall there)."""
        if self.ws is None or r.mode in ("out", "gone", "start") or getattr(r, "tilted", False):
            return
        p = self.pos(r)
        if p is None:
            return
        last = r.__dict__.get("felt_p")
        r.felt_p = p
        if last is None or math.dist(last, p) < 1.0 or math.dist(last, p) > 15.0:
            return
        if self.ws.proven(*p) and self.ws.proven(*last) and self.ws.state(*p) != 4 and self.ws.state(*last) != 4:
            return
        n = self.ws.felt_strip(last[0], last[1], p[0], p[1], FELT_HALF_W, self.unc(r) * 0.5)
        if n and (r.move or {}).get("kind") == "probe":
            self.ws.probe_cells += n

    def pick(self, r):
        """Menu first; then the fallback ladder. Returns (kind, cell, turn, L, v, traj)."""
        p, h = self.pos(r), self.heading(r)
        turn_ok = self.can_turn_here(r)
        if self.drill > 0 and turn_ok and self.rng.random() < self.drill:
            for cell in self.menu.order(r.name, free=True):
                turn, L, v = self.menu.draw(cell)
                tr = self.make(r, turn, L, v)
                e = tr.end
                if edge_dist(self.rect, *e) >= EDGE_CC and all(
                        math.dist(e, self.pos(o)) >= PLAN_CC + 5 for o in self.R.values()
                        if o is not r and self.pos(o) is not None):
                    return "drill", cell, turn, L, v, tr
        free = self.rng.random() < FREE_SHARE
        if turn_ok:
            for cell in self.menu.order(r.name, free=free):
                turn, L, v = self.menu.draw(cell)
                tr = self.make(r, turn, L, v)
                if self.feasible(r, tr):
                    return ("free" if free else "menu"), cell, turn, L, v, tr
        why = "no menu move fits" if turn_ok else "too near an edge to turn on the spot"
        # rung 1: off-menu, any heading, the longest that fits (shorter than the menu's least too)
        best = None
        for L in (70, 50, 35, 22, 14, 8, 5):
            hs = list(range(-180, 180, 15)) if turn_ok else [0]
            self.rng.shuffle(hs)
            for d in hs:
                tr = self.make(r, d, L, 7.5)
                if self.feasible(r, tr):
                    best = ("off-menu", None, d, L, 7.5, tr)
                    break
            if best:
                break
        if best:
            self.fallback(r, 1, "off-menu", why)
            return best
        # rung 2: escape to open space (board centre side), the longest feasible length, gap relaxed to the reflex's
        cx, cy = (self.rect[0] + self.rect[2]) / 2, (self.rect[1] + self.rect[3]) / 2
        h_c = math.degrees(math.atan2(cy - p[1], cx - p[0]))
        for dh in (0, 30, -30, 60, -60, 90, -90, 120, -120, 180):
            d = wrap180(h_c + dh - h) if turn_ok else 0.0
            for L in (40, 25, 15, 8, 4):
                tr = self.make(r, d, L, 5.0)
                if self.feasible(r, tr, gap_relax=PLAN_CC - REFLEX_CC, away=True):
                    self.fallback(r, 2, "escape", why + "; off-menu too")
                    return "escape", None, d, L, 5.0, tr
            if not turn_ok:
                break
        # deadlock: holding next to another holding robot -> the lower-priority one yields (others as they stand now)
        if r.hold_n >= 3:
            near = [o for o in self.R.values() if o is not r and o.mode in ("idle", "move") and o.hold_n >= 3 and
                    math.dist(self.pos(o), p) < 45.0]
            if near and all(o.rank < r.rank for o in near):
                ign = tuple(o.name for o in near)
                q = self.pos(near[0])
                h_a = math.degrees(math.atan2(p[1] - q[1], p[0] - q[0]))
                for dh in (0, 35, -35, 70, -70, 110, -110):
                    d = wrap180(h_a + dh - h) if turn_ok else 0.0
                    for L in (30, 18, 10, 5):
                        tr = self.make(r, d, L, 5.0)
                        if self.feasible(r, tr, gap_relax=PLAN_CC - REFLEX_CC, ignore=ign, away=True):
                            self.fallback(r, 3, "yield", "deadlock with %s: the lower-priority one moves away" %
                                          ", ".join(ign))
                            return "yield", None, d, L, 5.0, tr
        # rung 3: a holding turn on the spot (never leaves its own circle; only with that circle clear of everyone)
        room = all(math.dist(self.pos(o), p) >= HARD_CC + 1.0 + self.margin0(o) for o in self.R.values()
                   if o is not r and self.pos(o) is not None)
        if turn_ok and room:
            r.hold_sign = -r.hold_sign
            d = 40.0 * r.hold_sign
            tr = Traj.move(self.t + 0.2, p, h, d, 0.0, 5.0, m0=self.margin0(r))
            self.fallback(r, 4, "holding turn", why + "; no off-menu or escape move either")
            return "holding", None, d, 0.0, 5.0, tr
        if not turn_ok:
            # (11-05c) the edge trap (5 Nov: Right stood 3 min 7 cm from the edge): reverse away from the edge, else
            # turn to face the board's middle where the corners still clear the edge
            res = self.back_move(r, Ls=CORNER_BACK_CM, inward=True)
            if res is not None:
                self.fallback(r, 5, "reverse", why + ": backs away from the edge")
                return res
            if edge_dist(self.rect, *p) >= EDGE_TURN_MIN + (r.bubble if r.lost else 0.0) and room and \
                    self.keepout_ok(r, Traj.stand(self.t, *p, m=self.margin0(r))):
                d = wrap180(h_c - h)
                tr = Traj.move(self.t + 0.2, p, h, d, 0.0, 5.0, m0=self.margin0(r))
                self.fallback(r, 5, "edge turn", why + ": turns to face the middle (corners stay inside)")
                return "edge turn", None, d, 0.0, 5.0, tr
            res = self.corner_shuffle(r, p, h) if self.shuffle else None
            if res is not None:
                return res
        self.fallback(r, 6, "stand", "nothing fits and no room to turn on the spot: stands (trophy break)")
        return None

    def corner_shuffle(self, r, p, h):
        """(e10, C4) The corner trap's last rung. A robot a few cm from two sides, facing along neither, can find no
        move under the circle rules: a turn on the spot needs its whole half-diagonal circle clear, and every straight
        move loses room on one side (the e09 hung-robot case with the filter in charge stood for minutes in a corner).
        Here the body's FOUR CORNERS are checked exactly (the square, not the circle round it) through a small turn and
        a short slow drive, forward or back, kept SHUFFLE_MARGIN_CM + its uncertainty inside every side; the move must
        gain room (its middle further from the nearest side, or as far and further from the next one). Not when lost."""
        if r.lost is not None or p is None or h is None:
            return None
        m = SHUFFLE_MARGIN_CM + self.unc(r)

        def cmin(x, y, th):
            c, s_ = math.cos(math.radians(th)), math.sin(math.radians(th))
            b = BODY / 2.0
            return min(edge_dist(self.rect, x + c * a - s_ * q, y + s_ * a + c * q)
                       for a, q in ((b, b), (-b, b), (-b, -b), (b, -b)))

        def two(x, y):
            d = sorted((x - self.rect[0], self.rect[2] - x, y - self.rect[1], self.rect[3] - y))
            return d[0], d[1]
        e0, s0 = two(*p)
        # a body already closer than that may only move with every corner never worse than now (0.2 cm noise)
        m = min(m, cmin(p[0], p[1], h) - 0.2)
        best = None
        for dh in (0, 15, -15, 30, -30, 45, -45, 60, -60, 75, -75, 90, -90):
            n = max(1, int(abs(dh) / 3))
            if any(cmin(p[0], p[1], h + dh * k / n) < m for k in range(n + 1)):
                continue
            h1 = wrap180(h + dh)
            for sg in (1.0, -1.0):
                for L in (8.0, 6.0, 4.0, 3.0, 2.0):
                    ux, uy = sg * math.cos(math.radians(h1)), sg * math.sin(math.radians(h1))
                    k_ = int(L / 0.5)
                    if any(cmin(p[0] + ux * L * i / k_, p[1] + uy * L * i / k_, h1) < m for i in range(1, k_ + 1)):
                        continue
                    e1, s1 = two(p[0] + ux * L, p[1] + uy * L)
                    if not (e1 >= e0 + 0.3 or (e1 >= e0 - 0.05 and s1 >= s0 + 1.0)):
                        continue
                    score = 10.0 * (e1 - e0) + (s1 - s0) - 0.02 * abs(dh)
                    if best is None or score > best[0]:
                        best = (score, dh, sg, L)
        if best is None:
            return None
        _, dh, sg, L = best
        h1 = wrap180(h + dh)
        tr = Traj.move(self.t + 0.2, p, h1 if sg > 0 else wrap180(h1 + 180.0), 0.0, L, BACK_V, m0=self.margin0(r))
        if not self.keepout_ok(r, tr) or any(clearance_profile(tr, o.traj, now=self.t)[-1] < REFLEX_CC
                                             for o in self.others(r)):
            return None
        kind = "shuffle" if sg > 0 else "shuffle back"
        self.fallback(r, 5, kind, "corner trap: a %+.0f deg turn and %.0f cm %s, its corners kept %.1f cm inside" % (
            dh, L, "forward" if sg > 0 else "back", m))
        return kind, None, dh, L, BACK_V, tr

    def fallback(self, r, level, kind, why):
        self.counts["fallback"][kind] = self.counts["fallback"].get(kind, 0) + 1
        self.log("fallbacks", dict(t=round(self.t, 2), robot=r.name, level=level, kind=kind, why=why))

    def pose_agree(self, r, own, cam, th):
        """(11-05h, protection 1; the 4 h home run: Center's own pose 17 cm off the camera's, the plan from the camera,
        the drive from its own pose: into the x = 0 edge) Own pose vs camera (at the picture's time) over
        observe.POSE_AGREE_CM for POSE_AGREE_S: the robot is stopped and its pose re-seeded from the camera (fpose)
        before any further move."""
        import observe as OB
        w = r.__dict__.setdefault("pagree", OB.PoseAgree())
        ev = w.feed(self.t, own, cam, 0.0)
        if not ev:
            return
        self.counts["reseeds"] = self.counts.get("reseeds", 0) + 1
        h = th if th is not None else self.heading(r)
        self.event(r, "pose re-seeded", "its own pose was %.1f cm from the camera's for %.0f s: stopped, pose set from "
                   "the camera before any further move (observed)" % (ev["d_cm"], ev["for_s"]))
        if r.move is not None:
            self.halt(r, "own pose off the camera's")
            self.end_move(r)
        self.out(r, "fpose %.2f %.2f %.2f %.2f %.2f" % (cam[0], cam[1], h, 1.0, 3.0))
        r.cam_off = (0.0, 0.0)
        r.busy_until = max(r.busy_until, self.t + 1.0)

    def pose_fuse(self, r, own, cam, th, t_pic=None):
        """(e08, A1; 6 Nov run 1: Center's move 46 was planned 1.9 s after its own pose and the camera disagreed by 4.6
        cm, under the 2 s rule, and swung its front over the edge) Own pose vs camera, both at the picture's time: the
        disagreement widens its position uncertainty AT ONCE (unc(), so every plan and the edge rule see it), the next
        piece is planned from the fused position (plan_next), and when two fixes in a row agree with each other over
        POSE_AGREE_CM the robot's pose is re-seeded from the camera without stopping it. Logged (pose.csv)."""
        pf = r.__dict__.setdefault("pfuse", SR.PoseFuse(self.pose_rule))
        before = pf.disagree_cm
        ev = pf.feed(self.t, own, cam, 0.0)
        d = pf.disagree_cm
        if d > SR.SURE_TOL_CM and before <= SR.SURE_TOL_CM:
            self.counts["pose_doubts"] = self.counts.get("pose_doubts", 0) + 1
            self.log("pose", dict(t=round(self.t, 2), robot=r.name, what="doubt", d_cm=round(d, 2),
                                  unc_cm=round(self.unc(r), 2)))
        if not ev:
            return
        if not self.camera_proven:
            self.counts["reseeds_waiting"] = self.counts.get("reseeds_waiting", 0) + 1
            return                  # (e09) not before the camera is proven (a wrong camera name would pull every pose)
        self.counts["reseeds"] = self.counts.get("reseeds", 0) + 1
        h = th if th is not None else self.heading(r)
        q = self.projected(r, self.t, cam[0], cam[1], h) or (cam[0], cam[1], h)
        if t_pic is not None:       # (e17, 6) a measured drift: ev d_cm after its own driving since they last agreed
            self.drift_add(r, ev["d_cm"], self.own_path(r, r.__dict__.get("agree_t", t_pic - 10.0), t_pic), "re-seed")
        self.event(r, "pose re-seeded", "its own pose was %.1f cm from the camera's: uncertainty "
                   "widened at once; still off after %.0f s: pose set from the camera without stopping (observed)" % (
                       ev["d_cm"], ev["for_s"]))
        self.log("pose", dict(t=round(self.t, 2), robot=r.name, what="reseed", d_cm=ev["d_cm"],
                              unc_cm=round(self.unc(r), 2)))
        if RESEED_TURN and t_pic is not None:
            # (e17, 1e) from the picture's time, projected to now, never landing mid-turn (send_fpose)
            self.send_fpose(r, t_pic, cam[0], cam[1], th, 1.0, 3.0, "re-seed")
        else:
            self.out(r, "fpose %.2f %.2f %.2f %.2f %.2f" % (q[0], q[1], q[2] if q[2] is not None else h, 1.0, 3.0))
        r.cam_off = (0.0, 0.0)

    def on_raw_evidence(self, name, t, ax, front_cm, line_c):
        """(11-05h, 3') The robot's own sensors during a move, for the move's outcome: the accelerometer's jolts and
        vibration (ax), the front range (dC_cm), the middle line sensor."""
        r = self.R.get(name)
        if r is None or r.move is None:
            return
        ev = r.move.setdefault("ev", dict(ax=[], front=[], line=[]))
        ev["ax"].append(ax)
        ev["front"].append(front_cm)
        ev["line"].append(line_c)
        for k in ("ax", "front", "line"):
            del ev[k][:-400]

    def on_raw(self, name, t, roll, pitch, v_cms):
        """(11-05h, protection 2) The robot's own roll / pitch (RAW rows): a STANDING tilt over observe.STAND_TILT_DEG
        for STAND_TILT_S (Center sat at 3.4 deg, under the 5 deg tilt stop, for 64 min) is a hazard: a keep-out where
        it is, its move ended, and it may only back away when the camera shows room behind (else it stands, said)."""
        r = self.R.get(name)
        if r is None:
            return
        import observe as OB
        w = r.__dict__.setdefault("twatch", OB.TiltWatch())
        ev = w.feed(t, roll, pitch, v_cms)
        if getattr(r, "tilted", False) and w.level is not None and w.last <= w.lim and r.move is None and \
                r.__dict__.get("recover") is not None and r.recover.get("i", 0) > 0:
            self.recovered(r, "its tilt back to %.1f deg" % w.last)        # (e08, A3)
        if not ev:
            return
        self.counts["tilt_hazards"] = self.counts.get("tilt_hazards", 0) + 1
        if r.move is not None:
            self.halt(r, "standing tilt")
            self.end_move(r)
        self.tilt_trip(r, "standing tilt %.1f deg for %.1f s (a wheel over an edge?)" % (ev["dev_deg"], ev["for_s"]))

    # -- (e09, Phase B) the filter, the tracking layer's evidence, pictures --------------------------------------------
    GAP_FWD_S = 1.0          # CHOSEN: in a camera gap the filter's position goes to the robot this often (as a fix)
    GAP_FWD_MAX_CM = 8.0     # CHOSEN: ... only while the filter's uncertainty is under this (its dead reckoning beat
                             # the robot's own: 1.5 against 2.3 cm after 10 s on the 5 Nov logs)
    GYRO_FIX_S = 5.0         # CHOSEN: B3: the gyro heading goes to a standing robot at most this often ...
    GYRO_FIX_DEG = 1.0       # ... when its own heading is this far off it

    def ekf_live(self, r):
        """The filter is in charge of r's pose: pose_source "ekf", started, and its RAW rows flowing (a robot whose RAW
        stopped falls back to the one-step blend until they come again)."""
        fp = r.fp
        return self.pose_source == "ekf" and fp is not None and fp.started and fp.last_raw_t is not None and \
            self.t - fp.last_raw_t < 1.0

    def fp_fix(self, r, t, x, y, th, sigma=None, src=None):
        if r.fp is None:
            return                  # (the filter is made by the robot's first RAW row: no RAW, no filter)
        res = r.fp.fix(t, x, y, th, sigma, src or "camera", now=self.t, hold_disagree=not self.camera_proven)
        # (e09) a fix the filter held back (a disagreement while the camera is not proven) is not sent to the robot
        r.fix_used = bool(res is None or res.get("used", True)) or self.pose_source != "ekf"
        if res and res.get("disagree_cm", 0.0) > 0 and res.get("d2", 0.0) > FP.GATE_D2:
            self.counts["ekf_disagree"] = self.counts.get("ekf_disagree", 0) + 1
        r.pc = None

    def on_raw_full(self, name, t, v_cms, w_dps, gz_dps=None, still=None, ax=None, front_cm=None, line=None):
        """(e09, B2) One RAW row in true units (raw_units): the robot's wheels, raw gyro rate, accelerometer, front and
        line sensors, for its filter."""
        r = self.R.get(name)
        if r is None:
            return                  # (the filter runs in both pose sources: in "blend" beside it, logged only)
        if r.fp is None:
            r.fp = FP.FusedPose(r.name, gyro_heading=self.gyro_heading)
            if r.cam is not None:
                r.fp.start(r.cam[0], r.cam[1], r.cam[2], r.cam[3] if r.cam[3] is not None else self.heading(r),
                           1.0, 10.0 if r.cam[3] is not None else 30.0)
        r.fp.raw(t, v_cms, w_dps, gz_dps, still, ax, front_cm, line)
        if r.fp.started and r.fp.gh.th is None and r.own is not None:
            r.fp.gh.set(r.own[3])
        r.pc = None

    def gap_forward(self, r):
        """(e09, B2) In a camera gap the robot's own pose drifts by its wheels and library yaw; the filter's (wheels, raw
        gyro, learned wheel scale) drifts less: sent to the robot as a fix every GAP_FWD_S while it is sure enough."""
        if self.fix_fn is None or not self.ekf_live(r) or r.lost is None or self.t - r.gap_fwd_t < self.GAP_FWD_S or \
                not self.camera_proven:
            return                  # (not while the camera is not proven: the filter may have taken its picture)
        u = r.fp.unc_cm(self.t)
        if u is None or u > self.GAP_FWD_MAX_CM:
            return
        q = r.fp.pose(self.t)
        r.gap_fwd_t = self.t
        self.counts["gap_forwards"] = self.counts.get("gap_forwards", 0) + 1
        self.fix_fn(r.name, self.t, q[0], q[1])

    def gyro_watch(self, r):
        """(e09, B3, switch gyro_heading, OFF by default) A standing robot whose own heading (library yaw x gyro_scale)
        is off the raw gyro's heading is told the gyro's (fpose: its own position kept)."""
        if not self.gyro_heading or not self.ekf_live(r) or r.own is None or r.mode != "idle":
            return
        h = r.fp.gh.heading_deg()
        if h is None or self.t - r.gyro_fix_t < self.GYRO_FIX_S or abs(wrap180(h - r.own[3])) < self.GYRO_FIX_DEG:
            return
        r.gyro_fix_t = self.t
        self.counts["gyro_heading_fixes"] = self.counts.get("gyro_heading_fixes", 0) + 1
        self.out(r, "fpose %.2f %.2f %.2f %.2f %.2f" % (r.own[1], r.own[2], h, 1.0, 1.0))

    def identity_wiggle(self, r, why="identity", spots=None):
        """(e09, B1 wiggle first) The tracking layer cannot tell r from a robot it passed closely: r wiggles on its spot
        (the find-me wiggle) so the camera's turning outline names it. (e17, 1b) spots: the camera's candidate spots
        when r is unaccounted for (its own pose may be far off): it wiggles, held or not, only where a turn on the spot
        is safe at EVERY candidate (each spot TURN_EDGE_CC in from the edges, no other robot within PLAN_CC)."""
        if r.mode in ("out", "gone", "start"):
            return
        if spots:
            # a turn on the spot sweeps only its own circle (HALF): safe where every candidate spot is TURN_EDGE_CC in
            # from the edges and no robot the camera has now (fresh) is within two circles and the reflex gap
            seen = [(o.cam[1], o.cam[2]) for o in self.R.values() if o is not r and o.cam is not None and
                    o.mode != "gone" and self.fresh(o)]
            for sx, sy in spots:
                if edge_dist(self.rect, sx, sy) < TURN_EDGE_CC or any(
                        0.5 < math.dist(q, (sx, sy)) < 2 * HALF + REFLEX_GAP for q in seen + [
                            s_ for s_ in spots if s_ != (sx, sy)]):
                    self.counts["identity_wiggle_unsafe"] = self.counts.get("identity_wiggle_unsafe", 0) + 1
                    self.event(r, "identity wiggle", "%s: not safe at every place the camera might have it: it stands"
                               % why)
                    return
            if r.move is not None:
                self.halt(r, "identity wiggle")
                self.end_move(r)
            self.counts["identity_wiggles"] = self.counts.get("identity_wiggles", 0) + 1
            self.event(r, "identity wiggle", why)
            r.wiggle_due = 0.0
            return self.find_me(r, safe_checked=True)
        if getattr(r, "held", None):
            return
        if r.move is not None and r.move.get("kind") == "wiggle":
            return
        self.counts["identity_wiggles"] = self.counts.get("identity_wiggles", 0) + 1
        self.event(r, "identity wiggle", why)
        if r.move is not None:
            self.halt(r, "identity wiggle")
            self.end_move(r)
        r.wiggle_due = 0.0
        r.busy_until = self.t
        if r.mode == "idle" and self.pos(r) is not None:
            self.find_me(r)
        else:
            r.want_wiggle = self.t

    def on_obstacle(self, ob):
        """(e09, B1 brick watch) Something new stands on the table (the camera's "what changed" watch): a confirmed
        keep-out for every robot (the camera saw it: safety_rules.keepout_confirmed)."""
        q = dict(x=float(ob["x"]), y=float(ob["y"]), r=float(ob.get("r", 6.0)), t=round(self.t, 2), robot="-",
                 src=str(ob.get("src", "camera: new object")), edge_cm=round(edge_dist(self.rect, ob["x"], ob["y"]), 1),
                 d_cm=0.0, owner=None, confirmed=True)
        self.keepouts.append(q)
        self.counts["camera_objects"] = self.counts.get("camera_objects", 0) + 1
        self.log("keepouts", dict(q, what="new object seen by the camera"))
        self.event(None, "camera object", "something new at %.0f, %.0f: kept out" % (q["x"], q["y"]))

    def on_parked_moved(self, ev):
        """(e09, B1 parked watch) A robot whose wheels stood still was moved (pushed, picked up and put down): its pose
        is taken from the camera (the evidence wins), its filter re-seeded, the moment logged and a picture asked."""
        r = self.R.get(ev.get("robot"))
        if r is None:
            return
        self.counts["parked_moved"] = self.counts.get("parked_moved", 0) + 1
        self.event(r, "moved while parked", "the camera saw it move %.1f cm while its wheels stood: its pose is set "
                   "from the camera" % ev.get("d_cm", 0.0))
        h = self.heading(r)
        self.out(r, "fpose %.2f %.2f %.2f %.2f %.2f" % (ev["x"], ev["y"], h, 1.0, 5.0))
        r.fpose_t = self.t
        if r.fp is not None and r.fp.started:
            r.fp.reset(self.t, ev["x"], ev["y"], None, 1.0)
        r.cam_off = (0.0, 0.0)
        r.pc = None
        self.want_picture(r, "moved while parked")

    def want_picture(self, r, why):
        """(e09, B4) The host saves a camera picture of this moment into the session folder. (e10) At most one a robot
        each picture_every_s (Claude's default 60 s; e09 took 38-83 in a 9 min run with a parked robot)."""
        key = r.name if r else "-"
        last = self.__dict__.setdefault("picture_t", {})
        if self.picture_every_s > 0 and self.t - last.get(key, -1e9) < self.picture_every_s:
            self.counts["pictures_skipped"] = self.counts.get("pictures_skipped", 0) + 1
            return
        last[key] = self.t
        self.pictures.append(dict(t=round(self.t, 2), robot=r.name if r else "-", why=why))
        self.counts["pictures"] = self.counts.get("pictures", 0) + 1

    def take_pictures(self):
        p, self.pictures = self.pictures, []
        return p

    def border_fire(self, r, info):
        """(e08, Nick 7 Nov) the robots' own 6 cm virtual border is left as it is; how often it fires is counted and
        logged (the evidence to remove it one day: 0 in the 5 Nov 4 h run and the 6 Nov run)."""
        self.counts["border_fires"] = self.counts.get("border_fires", 0) + 1
        self.event(r, "virtual border", "its own border stopped the step (%s)" % info)

    def tilt_trip(self, r, info):
        """(11-05h, Nick 5 Nov 20:46: a robot seen tilted at the board's edge) Its own tilt stop ended the step: the
        spot just ahead is a discovered edge (a keep-out, as a front stop's), and the next move is a straight back-off,
        never forward (firmware 2026-11-05d refuses anything else while it is tilted; older firmware ends its follow
        run). Until it has backed away nothing else is planned for it."""
        self.counts["tilt_trips"] = self.counts.get("tilt_trips", 0) + 1
        self.event(r, "tilt stop", "its own tilt stop ended the step (%s): kept out ahead, backs straight away" % info)
        self.felt(r, "obstacle 0.0 cm (tilted)")
        p = self.pos(r)
        if p is not None:
            # (11-05h; Nick 21:38: nobody touched Center at 19:28, yet it moved while Left's front sensor read "Center"
            # 6.4 cm ahead) a tilted robot at an edge is kept out by the OTHERS with its whole body and a margin, so no
            # robot nudges it over; its own back-off may still leave (keep-outs allow moves that never get closer)
            q = dict(x=round(p[0], 1), y=round(p[1], 1), r=HALF + PLAN_GAP, t=round(self.t, 2), robot=r.name,
                     src="tilted robot", edge_cm=round(edge_dist(self.rect, *p), 1), d_cm=0.0, owner=r.name,
                     confirmed=True)
            self.keepouts.append(q)
            self.log("keepouts", dict(q, what="tilted robot kept clear by the others"))
        r.back_off = dict(t=self.t, h=self.heading(r), edge=True)
        r.tilted = True

    def start_move(self, r):
        res = None
        tilted = getattr(r, "tilted", False)
        if tilted and self.tilt_recovery:
            r.back_off = None
            return self.recover_step(r)            # (e08, A3) the recovery tries, never towards the drop
        if r.back_off is not None:
            bo_, r.back_off = r.back_off, None
            res = self.track_backout(r) if (EDGE_BACKOUT and bo_.get("edge")) else None   # (e16, 7) its own track
            if res is None:
                res = self.back_move(r)
            if res is None and tilted:
                res = self.back_move(r, inward=True)       # (11-05h) tilted at the edge: any back-off that gains room
            if res is not None:
                self.counts["backoffs"] += 1
                self.fallback(r, 0, "back off", "its own front stop: backs away before anything else (never the "
                                                "same heading again)")
                r.tilted = False
        if res is None and tilted:
            # (11-05h) tilted and no room to back: it stands (never a forward move or a turn); said, and tried again
            r.back_off = dict(t=self.t, h=self.heading(r))
            r.busy_until = self.t + 1.0
            if not r.stopped_hard:
                self.trophy_break(r, "tilted and no room to back away: stands until it can back off")
            return
        if res is None and r.__dict__.get("probe_ret"):
            res = self.probe_return(r)                 # (e10, C2) back from a probe
        if res is None:
            res = self.try_probe(r)                    # (e10, C2) the probe-ahead setting (off by default)
        if res is None:
            res = self.pick(r)
        if res is None and EDGE_BACKOUT:
            res = self.track_backout(r)                # (e16, 7) back along its own track, then re-plan
        if res is None:
            r.busy_until = self.t + 1.0
            if not r.stopped_hard:
                self.trophy_break(r, "nothing fits from a spot too near the edge to turn")
            return
        if r.stopped_hard and (r.adj is None or r.adj["kind"] != "stop"):
            self.close_break(r)          # it moves again: the full stop (if any) ends here
        kind, cell, turn, L, v, tr = res
        self.why_zero(r, kind if kind in ("back off", "reverse", "edge turn", "holding") else
                      "stop-turn (turns on the spot)")
        r.hold_n = r.hold_n + 1 if kind == "holding" else 0
        if cell is not None:
            self.menu.use(cell, r.name)
        p, h = self.pos(r), self.heading(r)
        h1 = wrap180(h + turn)
        rev = kind in ("back off", "reverse", "shuffle back", "probe return", "stray return back")
        mv = dict(kind=kind, cell=cell, turn=turn, L=L, v=v, t0=self.t, p0=p, h0=h, h1=h1, end=tr.end,
                  phase="turn", n=r.n_moves + 1, drive_id=None, turn_id=None, t_drive=None, adj_scale=1.0, rev=rev,
                  track_back=bool(getattr(tr, "track_back", False)))
        if abs(turn) >= 3.0:
            mv["turn_id"] = self.send_step(r, "fs %%d T %.2f -" % h1)
        if L > 0:
            ex, ey = tr.end
            lead = 0.0 if rev else SR.stop_lead_for(self.lead_tab.get(r.name), v, L)
            if lead > 0:
                # (e08, A5) its stop lead (from its calibration): sent that much short so it rests on the planned end
                ex, ey = p[0] + (ex - p[0]) * (L - lead) / L, p[1] + (ey - p[1]) * (L - lead) / L
                mv["lead_cm"] = round(lead, 2)
                self.counts["leads_used"] = self.counts.get("leads_used", 0) + 1
            mv["drive_id"] = self.send_step(r, "fs %%d D %.2f %.2f %.2f %.2f %.2f %.2f %d -" % (
                p[0], p[1], ex, ey, h1, v, 1 if rev else 0))
        else:
            mv["phase"] = "turn"
        if self.empty_move(r, mv, p, kind):
            return
        if kind == "probe":
            r.probe_active = dict(cells=r.__dict__.pop("probe_cells", []), p0=p, t=self.t)
        r.move, r.traj, r.mode = mv, tr, "move"
        r.n_moves += 1
        r.scale = 1.0
        self.pump(r)

    def empty_move(self, r, mv, p, kind):
        """(e10, C4) An EMPTY move (a turn under 3 deg and no drive: an "edge turn" for a robot that already faces the
        middle) sends nothing, so it used to wait for the 15 s slow-move timeout and be picked again: the e09 hung-robot
        case with the filter in charge stood 30 s this way. It ends at once; the robot picks again after a short pause
        (another robot's path may have moved on by then). True when the move was empty."""
        if mv["turn_id"] is not None or mv["drive_id"] is not None:
            return False
        self.counts["empty_moves"] = self.counts.get("empty_moves", 0) + 1
        if p is not None:
            r.traj = Traj.stand(self.t, *p, m=self.margin0(r))
        r.busy_until = self.t + EMPTY_MOVE_PAUSE_S
        if not r.stopped_hard:
            self.trophy_break(r, "%s: already facing that way, nothing to send; picks again" % kind)
        return True

    def step_over(self, r, sid, state, info):
        mv = r.move
        if mv is None:
            return
        if mv.get("kind") == "nudge":
            if sid == mv.get("drive_id") and state in ("done", "failed", "dropped"):
                self.nudge_over(r, mv, state)
            return
        if mv.get("kind") == "hdconfirm":                  # (e17, 2) the heading-confirm crawl ended
            if sid == mv.get("drive_id") and state in ("done", "failed", "dropped"):
                self.hdconfirm_over(r, mv, state)
            return
        if mv.get("curve"):
            return self.curve_step_over(r, sid, state, info)
        last = mv["drive_id"] or mv["turn_id"]
        if state == "done" and sid != last:
            return
        result = "done" if state == "done" else ("%s: %s" % (state, info))
        p = self.pos(r)
        end_err = math.dist(p, mv["end"]) if p and state == "done" else ""
        self.log("moves", dict(t=round(mv["t0"], 2), robot=r.name, n=mv["n"], kind=mv["kind"],
                               cell=Menu.label(mv["cell"]) if mv["cell"] else "",
                               turn_deg=round(mv["turn"], 1), length_cm=round(mv["L"], 1), speed=mv["v"],
                               x0=round(mv["p0"][0], 1), y0=round(mv["p0"][1], 1),
                               x1=round(mv["end"][0], 1), y1=round(mv["end"][1], 1),
                               est_s=round(turn_s(mv["turn"]) + (drive_s(mv["L"], mv["v"]) if mv["L"] else 0), 2),
                               dur_s=round(self.t - mv["t0"], 2), result=result,
                               end_err_cm=round(end_err, 2) if end_err != "" else "",
                               # (CLAUDE.md, 5 Nov: surface features found from the robots' own data) the robot's own
                               # pose against the camera where the move ended, with the camera's position: later tools
                               # can map where odometry disagrees (seams, bumps, slippery patches); no map is made here
                               # (11-05c) own pose at the PICTURE's time, not now (the fix is ~0.45 s old on the Pi)
                               own_vs_cam_cm=round(math.dist(self.own_at(r, r.cam[0])[:2], (r.cam[1], r.cam[2])), 2)
                               if (r.own and r.cam and r.lost is None and self.own_at(r, r.cam[0])) else "",
                               cam_x=round(r.cam[1], 1) if r.cam else "", cam_y=round(r.cam[2], 1) if r.cam else "",
                               style="stopturn", curvature=""))
        if state == "done" and self.fast_done(r, mv):
            state, info = "failed", "done without moving"
        if str(mv["kind"]).startswith("recover"):
            self.recover_result(r, mv, state, info)
        if state != "done":
            if "virtual border" in info:
                self.border_fire(r, info)
            if "tilted" in info:
                self.tilt_trip(r, info)
            elif "obstacle" in info or "plywood edge" in info:
                self.event(r, "front stop", "its own %s stopped the step: %s" % (
                    "front sensor" if "obstacle" in info else "edge guard", info))
                self.felt(r, info)
            elif "watchdog" in info:
                self.event(r, "watchdog", "its watchdog stopped the step (nothing heard from the Pi)")
            elif state == "failed":
                self.event(r, "step failed", info)
        self.end_move(r)

    ACCEL_JOLT_G = 0.03      # (3') the accelerometer "felt it roll": a jolt this far from its median (g) during the
                             # move. Chosen (no measured rolling vibration yet). DOE note: yes, RAW ax standing vs
                             # rolling vs wheels slipping on the bench sets it
    LINE_CHANGE = 15.0       # the middle line sensor's reading moved this much over the move (its noise ~5)

    def fast_done(self, r, mv):
        """(11-05h, 3' and 3b; Nick 21:11) A move reported done: did it happen? observe.move_outcome fuses the wheels
        (its own pose), the accelerometer, the front range, the line sensor, the time it took and, only when fresh, the
        camera; physical impossibility decides alone (Left's 4,289 reverses 'done in 0.05 s'). Not moved = a failure,
        remembered (FailMemory) so the same action is never repeated unchanged; failures piling up at one spot
        escalate. Moved with the camera blind but its own sensors agreeing: accepted, the bubble grown."""
        import observe as OB
        est = turn_s(mv["turn"]) + (drive_s(mv["L"], mv["v"]) if mv["L"] else 0.0)
        took = self.t - mv.get("t_drive", mv["t0"]) if mv.get("t_drive") else self.t - mv["t0"]
        if not mv["L"]:
            return False
        own0 = self.own_at(r, mv.get("t_drive") or mv["t0"])
        wheel = math.dist(own0[:2], (r.own[1], r.own[2])) if (own0 and r.own) else None
        cam_moved = None
        fresh = r.cam is not None and self.fresh(r) and r.lost is None
        if fresh and r.cam_hist:
            before = [q for q in r.cam_hist if q[0] <= mv["t0"]]
            q0 = before[-1] if before else r.cam_hist[0]
            cam_moved = math.dist(q0[1:], (r.cam[1], r.cam[2]))
        ev = mv.get("ev") or {}
        acc = None
        if len(ev.get("ax") or []) >= 5:
            a = sorted(ev["ax"])
            med = a[len(a) // 2]
            acc = max(abs(v - med) for v in a) >= self.ACCEL_JOLT_G
        fr = [v for v in ev.get("front") or [] if v is not None and 0 < v < 60]
        fchg = (fr[0] - fr[-1]) if len(fr) >= 4 else None
        fexp = (mv["L"] if not mv.get("rev") and mv["kind"] not in ("back off", "reverse") else -mv["L"]) \
            if fchg is not None else None
        ln = [v for v in ev.get("line") or [] if v is not None]
        lchg = (max(ln) - min(ln) >= self.LINE_CHANGE) if len(ln) >= 5 else None
        out = OB.move_outcome(mv["L"], took, wheel_cm=wheel, accel_rolling=acc, front_change_cm=fchg,
                              front_expect_cm=fexp, line_changed=lchg, cam_moved_cm=cam_moved, cam_fresh=fresh,
                              robot_info=mv.get("info", ""))
        mv["outcome"] = out
        if out["verdict"] == "moved":
            if out["blind_ok"] and r.lost is not None:
                self.grow_bubble(r)
            r.fast_n = 0
            return False
        self.counts["fast_done"] = self.counts.get("fast_done", 0) + 1
        p = self.pos(r) or (mv["p0"][0], mv["p0"][1])
        fm = r.__dict__.setdefault("fmem", OB.FailMemory())
        fm.add(self.t, mv["kind"], -1 if mv["kind"] in ("back off", "reverse") else 1, mv["L"], p[0], p[1])
        self.event(r, "did not move", "move %d (%s %.0f cm) reported done in %.2f s (expected %.1f): %s: counted as "
                   "NOT done; never repeated unchanged" % (mv["n"], mv["kind"], mv["L"], took, est, out["words"]))
        if fm.escalate(p[0], p[1]):
            self.hazard(r, p, "%d moves failed here" % OB.FailMemory.ESCALATE_N)
        return True

    def hazard(self, r, p, why):
        import observe as OB
        """(11-05h, 3b) Failures piling up at one spot: a keep-out there for everyone, its pose re-seeded from the
        camera when it has a fresh fix (else a find-me), and a full stop said, instead of the same action again."""
        q = dict(x=round(p[0], 1), y=round(p[1], 1), r=KEEP_R * 3, t=round(self.t, 2), robot=r.name, src="hazard",
                 edge_cm=round(edge_dist(self.rect, *p), 1), d_cm=0.0, confirmed=True)
        self.keepouts.append(q)
        self.log("keepouts", dict(q, what="hazard: " + why))
        self.counts["hazards"] = self.counts.get("hazards", 0) + 1
        self.event(r, "hazard", "%s at %.0f, %.0f: kept out for everyone; %s" % (
            why, p[0], p[1], "its pose re-seeded from the camera" if (r.cam and self.fresh(r)) else "find-me"))
        if r.cam is not None and self.fresh(r):
            self.out(r, "fpose %.2f %.2f %.2f %.2f %.2f" % (r.cam[1], r.cam[2], self.heading(r), 1.0, 3.0))
        r.fmem = OB.FailMemory()       # a fresh start after the hazard is marked
        r.busy_until = self.t + 2.0
        if not r.stopped_hard:
            self.trophy_break(r, "hazard: " + why)

    def end_move(self, r):
        p = self.pos(r)
        if r.move is not None and r.move.get("restore_tilt") is not None:
            # (e08, A3) a pivot ran with the tilt stop raised: back to the run's limit
            self.out(r, "set tilt_stop_deg %g" % r.move["restore_tilt"])
            r.move["restore_tilt"] = None
        r.move = None
        r.queue, r.coast = [], None
        r.controls = [c for c in r.controls if c["kind"] != "farc"]   # (11-05d) a bend for a piece that is over
        if r.adj is not None:
            self.close_adj(r, "move ended")
        r.scale = 1.0
        for st in r.steps.values():
            if st["status"] in ("queued_local", "sent", "acked", "started"):
                st["status"] = "dropped"
        if p is not None:
            r.traj = Traj.stand(self.t, *p, m=self.margin0(r))
        if r.mode == "move":
            r.mode = "idle"
        r.busy_until = self.t + PAUSE_S
        if r.mode == "parking":
            self.parked(r)

    def halt(self, r, why):
        self.why_zero(r, "halt: " + why)
        self.control(r, "halt", "halt")
        self.event(r, "halt", why)


    # -- (e08, A3) tilt recovery ----------------------------------------------------------------------------------
    RECOVER_L = 8.0          # cm: each reverse try (BACK_CM's longest)
    RECOVER_RETRY_S = 20.0   # a robot that has tried everything holds this long, then tries again from the start (the
                             # camera, its tilt or a person may have changed things). Chosen; no DOE

    def tilt_dev(self, r):
        w = r.__dict__.get("twatch")
        return (w.last if (w is not None and w.level is not None) else None)

    def recover_step(self, r):
        """(e08, A3; 6 Nov run 1: Center tilted 6.1 deg at the x = 0 edge, "no room to back away: stands", its 8 cm
        reverse "did NOT move") The next try of safety_rules.tilt_recovery_tries, each one logged (recovery.csv):
        straight back when backing goes inward by the camera and its own pose (or its trail), even when the general
        room check says no room; then a reverse one wheel's worth to each side; then a short inward pivot by the gyro
        with the tilt stop raised to the tilt now + 2 deg for that step only; hold only when nothing safe remains."""
        p, h = self.pos(r), self.heading(r)
        if p is None:
            return
        rc = r.__dict__.get("recover")
        if rc is not None and rc.get("hold_until") and self.t < rc["hold_until"]:
            r.busy_until = self.t + 1.0
            return
        if rc is None or rc.get("hold_until"):
            own = (r.own[1], r.own[2]) if r.own is not None else None
            fresh = r.cam is not None and self.fresh(r) and r.lost is None
            cam = (r.cam[1], r.cam[2]) if fresh else None
            trail_ok = bool(getattr(r, "trail", None) and r.trail.reverse_ok(self.RECOVER_L, r.bubble if r.lost
                                                                              else 0.0)[0])
            back_in = SR.inward_agreed(self.rect, own, cam if cam is not None else (own if trail_ok else None), h,
                                       reverse=True)
            inward = SR.inward_heading(self.rect, *(cam or own or p))
            rc = dict(t0=self.t, tries=SR.tilt_recovery_tries(h, inward, back_in), i=0, n=(rc or {}).get("n", 0) + 1,
                      inward=inward, back_in=back_in, trail=trail_ok)
            r.recover = rc
            self.event(r, "tilt recovery", "tilted %s at %.1f, %.1f (%.1f cm from the edge): tries %s (backing goes "
                       "inward: %s, by %s)" % ("%.1f deg" % self.tilt_dev(r) if self.tilt_dev(r) is not None else "",
                                               p[0], p[1], edge_dist(self.rect, *p), ", ".join(rc["tries"]),
                                               "yes" if back_in else "no", "camera and own pose" if cam else
                                               "its trail" if trail_ok else "nothing"))
        while rc["i"] < len(rc["tries"]):
            name = rc["tries"][rc["i"]]
            rc["i"] += 1
            res = self.recover_move(r, name, p, h)
            if res == "hold":
                break
            if res is None:
                self.log("recovery", dict(t=round(self.t, 2), robot=r.name, n=rc["n"], attempt=name, result="not "
                                          "safe from here: skipped", x=round(p[0], 1), y=round(p[1], 1),
                                          tilt=self.tilt_dev(r) if self.tilt_dev(r) is not None else ""))
                continue
            return
        # nothing safe remains: hold, said (and asked when a person is there); tried again later
        rc["hold_until"] = self.t + self.RECOVER_RETRY_S
        r.busy_until = self.t + 1.0
        self.counts["recover_holds"] = self.counts.get("recover_holds", 0) + 1
        self.log("recovery", dict(t=round(self.t, 2), robot=r.name, n=rc["n"], attempt="hold", result="nothing safe "
                                  "remains", x=round(p[0], 1), y=round(p[1], 1),
                                  tilt=self.tilt_dev(r) if self.tilt_dev(r) is not None else ""))
        if not r.stopped_hard:
            self.trophy_break(r, "tilted at the edge and no safe way out remains: holds (tries again in %.0f s)"
                              % self.RECOVER_RETRY_S)
        self.ask(r, "%s is tilted at the board's edge and could not back away safely. Please lift it back onto the "
                 "board, or say it is fine." % r.name, ("I moved it back", "Leave it"))

    def recover_move(self, r, name, p, h):
        """One try: sends it and returns True, or None when that try is not safe from here."""
        import observe as OB
        fm = r.__dict__.setdefault("fmem", OB.FailMemory())
        if name == "hold":
            return "hold"
        if name.startswith("reverse"):
            if not r.recover.get("back_in"):
                return None
            hr = math.radians(h)
            ux, uy = math.cos(hr), math.sin(hr)
            side = {"reverse_left": 1.0, "reverse_right": -1.0}.get(name, 0.0)

            def end_of(L_):
                return p[0] - L_ * ux - side * SR.ONE_SIDE_CM * uy, p[1] - L_ * uy + side * SR.ONE_SIDE_CM * ux
            L = self.RECOVER_L
            ex, ey = end_of(L)
            e0, e1 = edge_dist(self.rect, *p), edge_dist(self.rect, ex, ey)
            if e1 < EDGE_TURN_MIN + 0.5:
                # (e10, C4) near a corner the fixed length can leave it just short of turning room (the e09 hung-robot
                # case: freed 7.1 cm from a side, 7.7 needed, so it stood): up to RECOVER_EXTRA_CM further, the length
                # that ends furthest in
                for k in range(1, int(RECOVER_EXTRA_CM / 0.5) + 1):
                    q = end_of(self.RECOVER_L + 0.5 * k)
                    eq = edge_dist(self.rect, *q)
                    if eq > e1 + 0.05:
                        L, (ex, ey), e1 = self.RECOVER_L + 0.5 * k, q, eq
            # inward only: it ends further in, with its body on the board (not the general room check)
            if e1 < e0 + 1.0 or e1 < HALF + 1.0 + (r.bubble if r.lost else 0.0):
                return None
            if fm.same("recover:" + name, -1, L, p[0], p[1]):
                return None
            tr = Traj.move(self.t + 0.2, p, wrap180(h + 180.0), 0.0, math.dist(p, (ex, ey)), BACK_V,
                           m0=self.margin0(r))
            if not self.keepout_ok(r, tr) or any(clearance_profile(tr, o.traj, now=self.t)[-1] < REFLEX_CC
                                                 for o in self.others(r)):
                return None
            mv = self.recover_send(r, name, p, h, 0.0, math.dist(p, (ex, ey)), tr, rev=True, end=(ex, ey))
            return True
        if name.startswith("pivot"):
            sg = 1.0 if name == "pivot_left" else -1.0
            if fm.same("recover:" + name, 1, SR.PIVOT_DEG, p[0], p[1]):
                return None
            if any(math.dist(self.pos(o), p) < HARD_CC + 1.0 for o in self.R.values() if o is not r and
                   self.pos(o) is not None):
                return None
            lim = SR.tilt_limits()["tilt_stop_deg"]
            dev = self.tilt_dev(r)
            raise_to = min(SR.RECOVER_TILT_CAP, (dev if dev is not None else lim) + SR.RECOVER_TILT_MARGIN_DEG)
            self.out(r, "set tilt_stop_deg %g" % max(lim, raise_to))
            tr = Traj.move(self.t + 0.2, p, h, sg * SR.PIVOT_DEG, 0.0, 5.0, m0=self.margin0(r))
            mv = self.recover_send(r, name, p, h, sg * SR.PIVOT_DEG, 0.0, tr)
            mv["restore_tilt"] = lim
            return True
        return None

    def recover_send(self, r, name, p, h, turn, L, tr, rev=False, end=None):
        if r.stopped_hard and (r.adj is None or r.adj["kind"] != "stop"):
            self.close_break(r)
        h1 = wrap180(h + turn)
        mv = dict(kind="recover:" + name, cell=None, turn=turn, L=L, v=BACK_V, t0=self.t, p0=p, h0=h, h1=h1,
                  end=end or tr.end, phase="turn", n=r.n_moves + 1, drive_id=None, turn_id=None, t_drive=None,
                  adj_scale=1.0, rev=rev)
        if abs(turn) >= 3.0:
            mv["turn_id"] = self.send_step(r, "fs %%d T %.2f -" % h1)
        if L > 0:
            ex, ey = end or tr.end
            mv["drive_id"] = self.send_step(r, "fs %%d D %.2f %.2f %.2f %.2f %.2f %.2f %d -" % (
                p[0], p[1], ex, ey, h1, BACK_V, 1 if rev else 0))
        r.move, r.traj, r.mode = mv, tr, "move"
        r.n_moves += 1
        self.why_zero(r, "tilt recovery: " + name)
        self.counts["recover_tries"] = self.counts.get("recover_tries", 0) + 1
        self.pump(r)
        return mv

    def recover_result(self, r, mv, state, info):
        import observe as OB
        rc = r.__dict__.get("recover") or {}
        out = mv.get("outcome") or {}
        moved = state == "done" and out.get("verdict", "moved") == "moved"
        p = self.pos(r) or mv["p0"]
        self.log("recovery", dict(t=round(self.t, 2), robot=r.name, n=rc.get("n", 0), attempt=mv["kind"][8:],
                                  result=("moved" if moved else "did not move: %s" % (out.get("words") or info))[:120],
                                  x=round(p[0], 1), y=round(p[1], 1),
                                  tilt=self.tilt_dev(r) if self.tilt_dev(r) is not None else ""))
        if not moved:
            fm = r.__dict__.setdefault("fmem", OB.FailMemory())
            fm.add(self.t, mv["kind"], -1 if mv.get("rev") else 1, mv["L"] or abs(mv["turn"]), mv["p0"][0],
                   mv["p0"][1])
            return
        dev = self.tilt_dev(r)
        if mv.get("rev") and (dev is None or dev <= SR.tilt_limits()["stand_tilt_deg"]):
            self.recovered(r, "backed off inward (%s)" % mv["kind"][8:])

    def recovered(self, r, how):
        r.tilted = False
        r.recover = None
        self.counts["recovered"] = self.counts.get("recovered", 0) + 1
        self.event(r, "tilt recovered", "level again: %s" % how)
        self.log("recovery", dict(t=round(self.t, 2), robot=r.name, n=0, attempt="level", result=how,
                                  x=round(self.pos(r)[0], 1) if self.pos(r) else "",
                                  y=round(self.pos(r)[1], 1) if self.pos(r) else "", tilt=self.tilt_dev(r) or ""))

    # -- (e08, A2) sure / unsure near an edge ------------------------------------------------------------------------
    UNSURE_RECHECK_S = 2.0
    PERSON_SAYS_S = 60.0     # a person's answer stands this long at that spot (3 cm). Chosen; no DOE

    def attended_live(self):
        """(e16, 8d; home 08:4x: "Is it near the edge?" asked 5 times with nobody there) Attended, and nobody has left a
        question unanswered for ASK_WAIT_S: else the run decides by itself, as an unattended run does
        (MIRLT_ASK_UNATTENDED=1: as before). An answer makes it attended again."""
        if not self.attended:
            return False
        if ASK_UNATTENDED:
            return True
        return not self.__dict__.get("nobody_answers")

    def question_watch(self, r):
        """(e16, 8d) A question open longer than ASK_WAIT_S with no answer: nobody is watching. Said once, closed, and
        the run decides by itself from then on (until a person answers a question)."""
        q = r.__dict__.get("question")
        if not q or ASK_UNATTENDED or self.__dict__.get("nobody_answers"):
            return
        if self.t - r.__dict__.get("question_t", self.t) >= ASK_WAIT_S:
            self.nobody_answers = True
            self.counts["unanswered"] = self.counts.get("unanswered", 0) + 1
            self.event(r, "nobody answered", "no answer in %.0f s: the run decides by itself (as unattended) from now "
                       "on; an answer to a later question makes it ask again" % ASK_WAIT_S)
            self.close_question(r, "nobody answered: the run decided by itself")

    def ask(self, r, text, choices):
        """A question for a person: Mission Control's question card when the run is attended, else said and logged."""
        open_q = r.__dict__.get("question")
        if open_q is not None:
            return open_q
        if not self.attended_live():
            self.event(r, "decides by itself", "(not asked: nobody is answering) " + text)
            return None
        self.counts["questions"] = self.counts.get("questions", 0) + 1
        qid = None
        if self.attended and self.asks is not None:
            qid = self.asks.ask(r.name, text, choices)
        self.event(r, "question", ("asked: " if qid else "(not attended: logged only) ") + text)
        r.question = qid or "logged"
        r.question_t = self.t
        return r.question

    def answered(self, r):
        q = r.__dict__.get("question")
        if not q or q == "logged" or self.asks is None:
            return None
        a = self.asks.answer(q)
        if a is not None:
            r.question = None
            self.nobody_answers = False          # (e16, 8d) someone is there again
            self.event(r, "answer", "the person answered: %s" % a)
        return a

    def close_question(self, r, why):
        q = r.__dict__.get("question")
        if q and q != "logged" and self.asks is not None:
            self.asks.close(q, why)
        r.question = None

    def edge_watch(self, r):
        """(e08, A2) Every half second: is the software sure where r is relative to the edge? Logged on every change
        (edge_judgement.csv). Unsure (near an edge, and the camera is stale or its own pose disagrees with it): with a
        person there it is held and asked; alone it moves inward only along a direction both the camera and its own
        pose call inward, or holds; it never guesses outward."""
        if r.mode not in ("idle", "move") or getattr(r, "held", None) or getattr(r, "tilted", False):
            return
        p = self.pos(r)
        if p is None or r.lost is not None:
            return
        pf = r.__dict__.get("pfuse")
        dis = pf.disagree_cm if pf is not None else 0.0
        need = self.turn_need(r)
        margin = edge_dist(self.rect, *p) - need
        said = r.__dict__.get("person_says")
        if said and self.t - said[1] < self.PERSON_SAYS_S and math.dist(said[2], p) < 3.0:
            j = dict(state="clear" if said[0] == "clear" else "sure_near", why="a person said so %.0f s ago" % (
                self.t - said[1]))
        else:
            j = SR.edge_judgement(margin, self.fresh(r), dis)
        st = r.__dict__.get("edge_state")
        if j["state"] != st:
            r.edge_state = j["state"]
            self.log("edge_judgement", dict(t=round(self.t, 2), robot=r.name, state=j["state"], why=j["why"],
                                            margin_cm=round(margin, 2), need_cm=round(need, 2),
                                            unc_cm=round(self.unc(r), 2), disagree_cm=round(dis, 2),
                                            cam_fresh=int(self.fresh(r)), x=round(p[0], 1), y=round(p[1], 1)))
            self.counts.setdefault("judgements", {})
            self.counts["judgements"][j["state"]] = self.counts["judgements"].get(j["state"], 0) + 1
            if st == "unsure":
                self.close_question(r, "settled by the evidence: %s" % j["why"])
                r.unsure = None
                if r.stopped_hard and (r.adj is None or r.adj["kind"] != "stop"):
                    self.close_break(r)
        if j["state"] != "unsure":
            return
        if r.__dict__.get("unsure") is None:
            r.unsure = dict(t=self.t, why=j["why"], tried=0)
            self.event(r, "unsure near an edge", j["why"])
            mv = r.move
            # a move that ends further from the edges than it starts (a back-off, a reverse, an escape, its own inward
            # step) is never interrupted; one heading outward is stopped
            # (e10, run 2 incident a) "ends further in" by where it really goes, not by the plan's end (Right's planned
            # end was 32 cm in; it was heading for the edge)
            end_now = self.predicted_end(r) if mv is not None else None
            if r.mode == "move" and mv is not None and not str(mv["kind"]).startswith(("unsure", "recover")) and \
                    edge_dist(self.rect, *end_now) <= edge_dist(self.rect, *p) + 0.5:
                self.halt(r, "unsure near an edge")
                self.end_move(r)
            if self.attended_live():
                self.ask(r, "%s may be near the board's edge (%s). Is it near the edge?" % (r.name, j["why"]),
                         ("It is near the edge", "It is clear of the edge"))
            else:
                self.want_picture(r, "unsure near an edge (%s)" % j["why"])     # (e09, B4: Nick 7 Nov)
        a = self.answered(r)
        if a is not None:
            r.person_says = ("clear" if "clear" in a.lower() else "near", self.t, p)
        self.question_watch(r)                   # (e16, 8d)

    def unsure_step(self, r):
        """An idle robot that is unsure near an edge and alone: one short move that goes inward by BOTH the camera and
        its own pose (backwards first: it came from there), else it holds and checks again."""
        u = r.unsure
        self.question_watch(r)                          # (e16, 8d) nobody answering: it decides by itself
        if self.attended_live():
            r.busy_until = self.t + self.UNSURE_RECHECK_S
            if not r.stopped_hard:
                self.trophy_break(r, "unsure near an edge: held while a person is asked")
            return
        own = (r.own[1], r.own[2]) if r.own is not None else None
        cam = (r.cam[1], r.cam[2]) if (r.cam is not None and self.fresh(r) and self.camera_proven) else None
        p, h = self.pos(r), self.heading(r)
        if HEADING_CONFIRM and r.__dict__.get("hd_confirm") is not None and p is not None and any(
                math.dist(self.pos(o), p) < PLAN_CC + 6.0 for o in self.R.values()
                if o is not r and self.pos(o) is not None and o.mode != "gone"):
            # (e17, 2; 08:48) its heading is in doubt and a robot is near: no heading-dependent step (the reverse
            # that went toward Right); it holds and checks again
            r.busy_until = self.t + self.UNSURE_RECHECK_S
            self.counts["unsure_heading_doubt_holds"] = self.counts.get("unsure_heading_doubt_holds", 0) + 1
            if not r.stopped_hard:
                self.trophy_break(r, "unsure near an edge with its heading in doubt and a robot near: holds")
            return
        for rev in (True, False):
            if not SR.inward_agreed(self.rect, own, cam, h, reverse=rev):
                continue
            L = 6.0
            tr = Traj.move(self.t + 0.2, p, wrap180(h + 180.0) if rev else h, 0.0, L, BACK_V, m0=self.margin0(r))
            if edge_dist(self.rect, *tr.end) <= edge_dist(self.rect, *p) + 1.0 or not self.keepout_ok(r, tr):
                continue
            if any(clearance_profile(tr, o.traj, now=self.t)[-1] < REFLEX_CC for o in self.others(r)):
                continue
            mv = dict(kind="unsure: inward", cell=None, turn=0.0, L=L, v=BACK_V, t0=self.t, p0=p, h0=h, h1=h,
                      end=tr.end, phase="turn", n=r.n_moves + 1, drive_id=None, turn_id=None, t_drive=None,
                      adj_scale=1.0, rev=rev)
            mv["drive_id"] = self.send_step(r, "fs %%d D %.2f %.2f %.2f %.2f %.2f %.2f %d -" % (
                p[0], p[1], tr.end[0], tr.end[1], h, BACK_V, 1 if rev else 0))
            r.move, r.traj, r.mode = mv, tr, "move"
            r.n_moves += 1
            u["tried"] += 1
            self.counts["unsure_inward"] = self.counts.get("unsure_inward", 0) + 1
            self.event(r, "unsure near an edge", "moves %.0f cm %s: inward by the camera and its own pose" % (
                L, "back" if rev else "forward"))
            self.pump(r)
            return
        if EDGE_BACKOUT:
            res_t = self.track_backout(r)       # (e16, 7/8d) back along its own track: ground it has proven
            if res_t is not None:
                u["tried"] += 1
                self.stopturn_move(r, res_t)
                return
        r.busy_until = self.t + self.UNSURE_RECHECK_S
        self.counts["unsure_holds"] = self.counts.get("unsure_holds", 0) + 1
        if not r.stopped_hard:
            self.trophy_break(r, "unsure near an edge and no direction both the camera and its own pose call inward: "
                                 "holds and checks again")

    # -- (e08, A7) held robots: never out of the run --------------------------------------------------------------
    def hold_robot(self, r, kind, why):
        """A robot that is silent for long, or whose heading is not known: held as an obstacle where the camera sees it,
        checked again every safety_rules.CHECK_EVERY_S, rejoins when the camera and its own pose agree again."""
        if getattr(r, "held", None) and r.held["kind"] == kind:
            return
        p = self.pos(r)
        r.held = dict(kind=kind, why=why, t=self.t, check=self.t + 1.0, agree_since=None, n=0)
        if r.move is not None:
            self.halt(r, "held: " + why)
            self.end_move(r)
        if p is not None:
            r.traj = Traj.stand(self.t, *p, m=self.margin0(r) + 1.0)
        self.counts["held"] = self.counts.get("held", 0) + 1
        self.event(r, "held", "%s: held as an obstacle where the camera sees it, checked again every %.0f s; it "
                   "rejoins when the camera and its own pose agree" % (why, SR.CHECK_EVERY_S))

    def held_check(self, r):
        hd = r.held
        t = self.t
        if r.cam is not None and self.fresh(r):
            r.traj = Traj.stand(t, r.cam[1], r.cam[2], m=self.margin0(r) + 1.0)
        heard = r.heard is not None and t - r.heard < 2.0 and not r.silent
        fresh = r.cam is not None and self.fresh(r)
        pf = r.__dict__.get("pfuse")
        agree = heard and fresh and r.own is not None and t - r.own[0] < 1.0 and \
            (pf is None or pf.disagree_cm <= SR.SURE_TOL_CM)
        if hd["kind"] == "heading":
            if r.move is not None:
                return
            if agree and hd.get("heading_ok"):
                return self.rejoin(r, "its heading measured by a %.0f cm nudge the camera saw" % hd["nudged"])
            if t < hd["check"]:
                return
            hd["check"] = t + SR.CHECK_EVERY_S
            hd["n"] += 1
            if not (heard and fresh):
                self.log("held", dict(t=round(t, 2), robot=r.name, kind="heading", check=hd["n"], result="not heard"
                                      " or not seen"))
                return
            return self.heading_nudge(r)
        # silent
        if not agree:
            hd["agree_since"] = None
            if heard and fresh and r.own is not None and pf is not None and pf.disagree_cm > SR.SURE_TOL_CM and \
                    t >= hd["check"]:
                hd["check"] = t + 5.0
                self.out(r, "fpose %.2f %.2f %.2f 1.0 3.0" % (r.cam[1], r.cam[2], self.heading(r)))
                self.event(r, "held", "heard again; its own pose %.1f cm off the camera's: re-seeded from the camera"
                           % pf.disagree_cm)
            elif t >= hd["check"]:
                hd["check"] = t + SR.CHECK_EVERY_S
                hd["n"] += 1
                self.log("held", dict(t=round(t, 2), robot=r.name, kind=hd["kind"], check=hd["n"],
                                      result="heard" if heard else "still silent"))
                if heard:
                    self.out(r, "status")
            return
        if hd["agree_since"] is None:
            hd["agree_since"] = t
        if t - hd["agree_since"] >= SR.AGREE_FOR_S:
            self.rejoin(r, "heard again; camera and its own pose agree for %.0f s" % SR.AGREE_FOR_S)

    def rejoin(self, r, why):
        hd = r.held
        r.held = None
        r.mode = "idle"
        r.busy_until = self.t + 0.5
        self.counts["rejoined"] = self.counts.get("rejoined", 0) + 1
        self.log("held", dict(t=round(self.t, 2), robot=r.name, kind=hd["kind"] if hd else "", check="",
                              result="rejoined after %.0f s: %s" % (self.t - (hd["t"] if hd else self.t), why)))
        self.event(r, "rejoined", "%s (held %.0f s)" % (why, self.t - (hd["t"] if hd else self.t)))

    def heading_nudge(self, r):
        """A held robot whose heading is not known (its heading crawl failed): a 6 cm nudge the camera watches, only
        where 6 cm in ANY direction keeps the derived clearance (the heading is not known)."""
        import observe as OB
        p = (r.cam[1], r.cam[2])
        need = self.need_cm(r, BACK_V, True) + OB.NUDGE_CM
        if edge_dist(self.rect, *p) < need or any(math.dist(self.pos(o), p) < PLAN_CC + OB.NUDGE_CM
                                                  for o in self.R.values() if o is not r and self.pos(o) is not None):
            self.log("held", dict(t=round(self.t, 2), robot=r.name, kind="heading", check=r.held["n"],
                                  result="no room for a nudge in every direction here (%.0f cm from the edge)"
                                  % edge_dist(self.rect, *p)))
            return
        h = self.heading(r)
        tr = Traj.move(self.t + 0.2, p, h, 0.0, OB.NUDGE_CM, BACK_V, m0=self.margin0(r) + OB.NUDGE_CM)
        mv = dict(kind="nudge", cell=None, turn=0.0, L=OB.NUDGE_CM, v=BACK_V, t0=self.t, p0=p, h0=h, h1=h,
                  end=tr.end, phase="turn", n=r.n_moves + 1, drive_id=None, turn_id=None, t_drive=None,
                  adj_scale=1.0, rev=False, cam0=(r.cam[1], r.cam[2], h))
        mv["drive_id"] = self.send_step(r, "fs %%d D %.2f %.2f %.2f %.2f %.2f %.2f 0 -" % (
            p[0], p[1], tr.end[0], tr.end[1], h, BACK_V))
        r.move, r.traj = mv, tr
        r.n_moves += 1
        self.pump(r)

    def nudge_over(self, r, mv, state):
        """The heading nudge ended: the camera's motion direction is the heading (observe.heading_offset); fpose."""
        import observe as OB
        r.move = None
        for st in r.steps.values():
            if st["status"] in ("queued_local", "sent", "acked", "started"):
                st["status"] = "dropped"
        if r.cam is None or not self.fresh(r):
            return
        res = OB.heading_offset(mv["cam0"], (r.cam[1], r.cam[2], mv["cam0"][2]))
        self.log("held", dict(t=round(self.t, 2), robot=r.name, kind="heading", check=r.held["n"] if r.held else "",
                              result="nudge %s: %s" % (state, "camera saw %.1f cm" % res[2] if res else
                                                       "the camera saw under 3 cm")))
        if res is None:
            return
        snapped, raw, moved = res
        h = wrap180(mv["cam0"][2] + raw)
        self.out(r, "fpose %.2f %.2f %.2f 1.0 3.0" % (r.cam[1], r.cam[2], h))
        r.own = (self.t, r.cam[1], r.cam[2], h) + tuple((r.own or (0, 0, 0, 0, "idle", 0))[4:])
        if r.held is not None:
            r.held["heading_ok"] = True
            r.held["nudged"] = moved
        self.event(r, "heading found", "a %.0f cm nudge: the camera saw it go towards %.0f deg (its own idea was %.0f):"
                   " its heading set" % (moved, h, mv["cam0"][2]))

    # -- (e08, A5) heading watch ----------------------------------------------------------------------------------
    def heading_check(self, r, x, y):
        """The robot's OWN heading checked against the direction the camera sees it drive, on every straight stretch
        (observe.HeadingWatch, run_plan's watch): two disagreeing checks in a row re-set its heading (fpose) from the
        motion. Heading lost after a restart or a sleep was seen 5 times (7 Nov audit)."""
        import observe as OB
        if not self.heading_watch or r.own is None:
            return
        w = r.__dict__.setdefault("hwatch", OB.HeadingWatch())
        mv = r.move
        sign = 0
        if mv is not None and mv["phase"] == "drive" and r.lost is None and (not mv.get("curve") or
                                                                              abs(mv.get("k", 0.0)) < 1e-3):
            sign = -1 if mv.get("rev") else 1
        ev = w.feed(self.t, (x, y, r.own[3]), r.own[3], sign)
        if not ev:
            return
        self.counts.setdefault("heading", {})
        self.counts["heading"][ev["kind"]] = self.counts["heading"].get(ev["kind"], 0) + 1
        if ev["kind"] == "heading_ok":
            return
        self.log("heading", dict(t=round(self.t, 2), robot=r.name, **{k: v for k, v in ev.items()}))
        if ev["kind"] == "heading_relearned":
            h = wrap180(r.own[3] + ev["new_deg"])
            r.hwatch = OB.HeadingWatch()
            self.out(r, "fpose %.2f %.2f %.2f 1.0 3.0" % (x, y, h))
            self.event(r, "heading re-set", "twice in a row it drove %.0f deg off its own heading (camera): its heading "
                       "set to %.0f" % (ev["err_deg"], h))
            if HEADING_CONFIRM:     # (e17, 2) the next move waits for a camera-confirmed heading
                r.hd_confirm = dict(t=self.t, why="heading re-set (%.0f deg)" % ev["err_deg"], tries=0, stage=None)
            if r.move is not None:
                self.halt(r, "heading re-set")
                self.end_move(r)

    # -- trophy breaks (full stops) ----------------------------------------------------------------------------
    def trophy_break(self, r, why):
        self.counts["breaks"] += 1
        r.stopped_hard = dict(t=self.t, why=why)
        self.why_zero(r, "trophy break: " + why)
        self.event(r, "trophy break", why)

    def close_break(self, r):
        b = r.stopped_hard
        if b:
            self.log("breaks", dict(t=round(b["t"], 2), robot=r.name, why=b["why"], dur_s=round(self.t - b["t"], 2)))
        r.stopped_hard = None

    # -- the reflex --------------------------------------------------------------------------------------------
    def motion(self, r, scale=None, steer=None):
        """(p, u, v, rem): where it is now (camera, moved on by the fix's age), unit direction, speed, cm left."""
        p = self.pos(r)
        mv = r.move
        if p is None or mv is None or mv["phase"] != "drive" or r.lost_hold:
            return p, (0.0, 0.0), 0.0, 0.0
        if r.own is not None and (r.own[4] != "drive" or self.t - r.own[0] > 0.6):
            return p, (0.0, 0.0), 0.0, 0.0
        end = steer if steer is not None else mv["end"]
        dx, dy = end[0] - p[0], end[1] - p[1]
        rem = math.hypot(dx, dy)
        if rem < 0.5:
            return p, (0.0, 0.0), 0.0, 0.0
        u = (dx / rem, dy / rem)
        v = mv["v"] * (r.scale if scale is None else scale)
        if r.cam and r.lost is None and not getattr(r, "pc_proj", False):   # (11-05c) a projected pos is now already
            age = max(0.0, self.t - r.cam[0])
            adv = min(rem, v * age)
            p = (p[0] + u[0] * adv, p[1] + u[1] * adv)
            rem -= adv
        return p, u, v, rem

    @staticmethod
    def path(m, H=REFLEX_H_S, dt=REFLEX_STEP_S):
        p, u, v, rem = m
        ts = np.arange(0.0, H + 1e-9, dt)
        s = np.minimum(v * ts, rem)
        return np.stack([p[0] + u[0] * s, p[1] + u[1] * s], axis=1), ts

    def pair_min(self, ma, mb):
        """Closest predicted distance and when; a pair that is not closing in (its closest is now, or within 0.3 cm
        of now) comes back as +inf: moving apart is the cure, never a reason to stop."""
        pa, ts = (ma, np.arange(0.0, REFLEX_H_S + 1e-9, REFLEX_STEP_S)) if isinstance(ma, np.ndarray) else self.path(ma)
        pb = mb if isinstance(mb, np.ndarray) else self.path(mb)[0]
        d = np.hypot(pa[:, 0] - pb[:, 0], pa[:, 1] - pb[:, 1])
        k = int(np.argmin(d))
        if d[k] >= d[0] - 0.3:
            return float("inf"), float(ts[k])
        return float(d[k]), float(ts[k])

    def pred(self, o):
        """(11-05d) A robot's predicted motion: a curve robot's positions over REFLEX_H_S, else the motion tuple."""
        return self.curve_pts(o)[0] if (o.curve and (o.move is None or o.move.get("curve"))) else self.motion(o)

    def need(self, a, b):
        return REFLEX_CC + (a.bubble if a.lost else 0.0) + (b.bubble if b.lost else 0.0) + \
            self.pinned_extra(a) + self.pinned_extra(b)          # (e16, 8b) a pinned robot's doubt

    def worst_for(self, r, mr, skip=(), hedge=True):
        """The worst predicted (gap - need) of r's motion mr against everyone else's: each other robot both as
        commanded and standing where the camera has it (a robot that stopped answering, a stop not yet reported)."""
        worst, who, tmin = float("inf"), None, None
        for o in self.R.values():
            if o is r or o.name in skip or self.pos(o) is None or o.mode in ("gone",):
                continue
            mo = self.pred(o)
            d, tm = self.pair_min(mr, mo)
            if hedge and self.moving(o):
                d2, tm2 = self.pair_min(mr, (self.pos(o), (0.0, 0.0), 0.0, 0.0))
                if d2 < d:
                    d, tm = d2, tm2
            g = d - self.need(r, o)
            if g < worst:
                worst, who, tmin = g, o, tm
        return worst, who, tmin

    def reflex(self):
        movers = [r for r in self.R.values() if r.mode in ("move", "parking") and r.move is not None]
        if not movers:
            return
        seen = set()
        for r in sorted(movers, key=lambda q: -q.rank):          # lowest priority first
            if r.adj is not None:
                continue
            if r.curve and r.move is not None and r.move.get("curve"):
                if not self.moving(r):
                    continue
                mr = self.curve_pts(r)[0]
            else:
                mr = self.motion(r)
                if mr[2] <= 0:
                    continue
            g, o, tm = self.worst_for(r, mr, hedge=True)
            if o is None or g >= 0 or (r.name, o.name) in seen:
                continue
            seen.add((o.name, r.name))
            # who acts: the lower-priority mover (r is the lower one unless o moves and ranks lower)
            actors = [r]
            if self.moving(o) and o.adj is None and (o.move is not None):
                actors = sorted([r, o], key=lambda q: -q.rank)      # the lower-priority mover first
            pred_before = g + self.need(r, o)
            done = False
            for a in actors:
                if a.curve and a.move is not None and a.move.get("curve"):
                    ok = self.try_adjust_curve(a, o if a is r else r, pred_before)
                else:
                    ok = self.try_adjust(a, o if a is r else r, pred_before)
                if ok:
                    done = True
                    break
            if done:
                continue
            a = actors[0]
            if a.curve and a.move is not None and a.move.get("curve"):
                if pred_before < HARD_CC + (r.bubble if r.lost else 0) + (o.bubble if o.lost else 0) + \
                        self.pinned_extra(r) + self.pinned_extra(o) and tm <= HARD_T_S:
                    self.curve_stop(a, o if a is r else r, pred_before)
                continue
            hard = pred_before < HARD_CC + (r.bubble if r.lost else 0) + (o.bubble if o.lost else 0) + \
                self.pinned_extra(r) + self.pinned_extra(o) and tm <= HARD_T_S
            if hard:
                self.apply(a, o if a is r else r, "stop", 0.0, None, pred_before, pred_before)
            else:
                self.replan(a, o if a is r else r, pred_before)

    def try_adjust(self, a, other, pred_before, speeds=True):
        """Speed first (only against a robot that moves: slowing towards one that stands only delays it), then a
        steer; the first that keeps every predicted gap is applied. False: none does."""
        cands = []
        other_moves = self.moving(other)
        for s in (SPEED_OPTIONS if (speeds and other_moves) else ()):
            if a.move["v"] * s > V_MAX or abs(s - a.scale) < 0.01:
                continue
            cands.append(("speed", s, None))
        p, u, v, rem = self.motion(a)
        if rem > 3.0:
            h = math.degrees(math.atan2(u[1], u[0]))
            for dh in STEER_OPTIONS:
                hh = math.radians(h + dh)
                L = min(rem, 30.0)
                end = (p[0] + math.cos(hh) * L, p[1] + math.sin(hh) * L)
                if edge_dist(self.rect, *end) < self.end_need(a):
                    continue
                cands.append(("steer", 1.0, end))
        for kind, s, end in cands:
            m = self.motion(a, scale=s, steer=end)
            g, o2, _ = self.worst_for(a, m)
            if g >= 0.5:
                if kind == "steer":
                    tr = Traj.move(self.t, p, h, 0.0, math.dist(p, end), a.move["v"], m0=self.margin0(a))
                    tr.end = end
                    if not self.feasible(a, tr, gap_relax=PLAN_CC - REFLEX_CC):
                        continue
                self.apply(a, other, kind, s, end, pred_before, g + self.need(a, o2) if o2 else 99.0)
                return True
        return False

    def apply(self, a, other, kind, scale, end, pred_before, pred_after):
        self.ev_id += 1
        method = "fspd" if a.new_fw else "reissue"
        if kind == "steer":
            method = "fsteer" if a.new_fw else "reissue"
        ev = dict(id=self.ev_id, t=self.t, actor=a.name, other=other.name, kind=kind, scale=scale,
                  steer=end, pred_before=pred_before, pred_after=pred_after, method=method, clear_since=None,
                  min_cc=float("inf"))
        a.adj = ev
        self.counts["reflex"][kind] = self.counts["reflex"].get(kind, 0) + 1
        if kind == "stop":
            self.trophy_break(a, "reflex hard stop: %s predicted %.1f cm from %s in %.1f s" % (
                a.name, pred_before, other.name, HARD_T_S))
        self.send_motion(a, scale, end)
        a.scale = scale
        if end is not None:
            a.move["end"] = end
            a.move["h1"] = math.degrees(math.atan2(end[1] - self.pos(a)[1], end[0] - self.pos(a)[0]))
        self.retraj(a)

    def replan(self, a, other, pred_before):
        """Neither a speed nor a steer keeps the gap: this drive ends and the planner picks a new move from here (a
        turn on the spot first: the wheels keep turning). Logged as a reflex event of kind replan."""
        self.ev_id += 1
        self.counts["reflex"]["replan"] = self.counts["reflex"].get("replan", 0) + 1
        mc = math.dist(self.pos(a), self.pos(other))
        self.log("reflex", dict(id=self.ev_id, t=round(self.t, 2), actor=a.name, other=other.name, kind="replan",
                                scale="", steer="", pred_before=round(pred_before, 2), pred_after="",
                                method="halt+new move", dur_s=0.0, actual_min_cc=round(mc, 2),
                                outcome="kept" if mc >= REFLEX_CC - 1.0 else (
                                    "closer than the gap, no touch" if mc >= HARD_CC else "circles touched"),
                                closed="replanned"))
        self.halt(a, "reflex: no speed or steer keeps %.1f cm from %s: new move" % (REFLEX_CC, other.name))
        self.end_move(a)
        a.busy_until = self.t

    def send_motion(self, a, scale, end):
        if a.new_fw:
            if end is not None:
                self.control(a, "fsteer %.2f %.2f" % end, "fsteer")
            else:
                self.control(a, "fspd %.2f" % scale, "fspd")
            return
        # older firmware: halt, then the rest of the drive as a new step at the new speed / to the new end
        self.control(a, "halt", "halt")
        mv = a.move
        for st in a.steps.values():
            if st["status"] in ("queued_local", "sent", "acked", "started"):
                st["status"] = "dropped"
        if scale <= 0.0:
            mv["drive_id"] = None
            mv["paused"] = True
            return
        p = self.pos(a)
        tgt = end if end is not None else mv["end"]
        h = math.degrees(math.atan2(tgt[1] - p[1], tgt[0] - p[0]))
        mv["drive_id"] = self.send_step(a, "fs %%d D %.2f %.2f %.2f %.2f %.2f %.2f 0 -" % (
            p[0], p[1], tgt[0], tgt[1], h, max(2.5, mv["v"] * scale)))
        mv["phase"] = "drive"
        self.pump(a)

    def retraj(self, a):
        mv = a.move
        p = self.pos(a)
        if mv is None or p is None:
            return
        L = math.dist(p, mv["end"])
        v = max(0.5, mv["v"] * a.scale)
        if a.scale <= 0.0:
            tr = Traj([(self.t, self.t + 3.0, p[0], p[1], self.margin0(a), SLACK0_S)])
            tr2 = Traj.move(self.t + 3.0, p, mv["h1"], 0.0, L, mv["v"], m0=self.margin0(a))
            a.traj = Traj(np.vstack([np.stack([tr.lo, tr.hi, tr.x, tr.y, tr.m, tr.sl], 1),
                                     np.stack([tr2.lo, tr2.hi, tr2.x, tr2.y, tr2.m, tr2.sl], 1)]))
            a.traj.end = mv["end"]
            return
        h = math.degrees(math.atan2(mv["end"][1] - p[1], mv["end"][0] - p[0]))
        a.traj = Traj.move(self.t, p, h, 0.0, L, v, m0=self.margin0(a))

    def adj_watch(self):
        for a in self.R.values():
            ev = a.adj
            if ev is None:
                continue
            o = self.R[ev["other"]]
            pa, po = self.pos(a), self.pos(o)
            if pa and po:
                ev["min_cc"] = min(ev["min_cc"], math.dist(pa, po))
            if a.move is None:
                self.close_adj(a, "move ended")
                continue
            if a.curve and a.move.get("curve"):
                self.adj_watch_curve(a, ev)
                continue
            if ev["kind"] == "steer":
                self.close_adj(a, "steered")
                continue
            m = self.motion(a, scale=1.0)
            if a.move.get("paused"):
                p = self.pos(a)
                m = (p, m[1], a.move["v"], math.dist(p, a.move["end"])) if m[2] == 0 else m
                if m[2] == 0:
                    p = self.pos(a)
                    e = a.move["end"]
                    rem = math.dist(p, e)
                    u = ((e[0] - p[0]) / rem, (e[1] - p[1]) / rem) if rem > 0.5 else (0.0, 0.0)
                    m = (p, u, a.move["v"], rem)
            g, _, _ = self.worst_for(a, m)
            if ev["kind"] == "stop" and g < 2.0 and self.t - ev["t"] >= 0.6:
                # a full stop lasts only until the planner has a move that goes away (Nick: no stops)
                o = self.R[ev["other"]]
                self.halt(a, "after the hard stop: a new move away from %s" % o.name)
                self.end_move(a)
                a.busy_until = self.t
                continue
            if g >= 2.0:
                if ev["clear_since"] is None:
                    ev["clear_since"] = self.t
                elif self.t - ev["clear_since"] >= CLEAR_HOLD_S:
                    a.scale = 1.0
                    if a.new_fw:
                        self.control(a, "fspd 1.00", "fspd")
                    else:
                        if a.move.get("paused"):
                            a.move.pop("paused", None)
                        self.send_motion(a, 1.0, None)
                    self.retraj(a)
                    self.close_adj(a, "clear")
            else:
                ev["clear_since"] = None
                if ev["kind"] == "speed" and self.t - ev["t"] > 0.4:
                    gc, oc, tm = self.worst_for(a, self.motion(a))
                    if gc < 0 and oc is not None:
                        # the speed in force no longer keeps the gap: escalate (steer, then a new move; a hard stop
                        # only when the circles would touch within HARD_T_S)
                        pb = gc + self.need(a, oc)
                        self.close_adj(a, "escalated")
                        if pb < HARD_CC and tm <= HARD_T_S:
                            self.apply(a, oc, "stop", 0.0, None, pb, pb)
                        elif not self.try_adjust(a, oc, pb):
                            self.replan(a, oc, pb)

    def close_adj(self, a, why):
        ev = a.adj
        a.adj = None
        if ev is None:
            return
        if ev["kind"] == "stop":
            self.close_break(a)
        mc = ev["min_cc"]
        out = "kept" if mc >= REFLEX_CC - 1.0 else ("closer than the gap, no touch" if mc >= HARD_CC else
                                                     "circles touched")
        self.log("reflex", dict(id=ev["id"], t=round(ev["t"], 2), actor=ev["actor"], other=ev["other"],
                                kind=ev["kind"], scale=ev["scale"],
                                steer="" if ev["steer"] is None else "%.1f;%.1f" % tuple(ev["steer"]),
                                pred_before=round(ev["pred_before"], 2), pred_after=round(min(ev["pred_after"], 99), 2),
                                method=ev["method"], dur_s=round(self.t - ev["t"], 2),
                                actual_min_cc=round(mc, 2) if mc < 1e9 else "", outcome=out, closed=why))

    # -- (11-05d) continuous curves --------------------------------------------------------------------------------
    def uses_curves(self, r):
        return self.motion_style == "curves" and bool(r.fw) and r.fw >= ARC_FW

    def why_zero(self, r, why):
        r.zero_why = (why, self.t)

    def curve_ok(self, r, tr, gap_relax=0.0, away=False, ignore=(), edge_away=False):
        """A curve plan (samples + loop disc) keeps the edges (every sample's middle LOOP_EDGE in, plus its margin
        beyond the base; the loop disc inside), the keep-outs and everyone's reservations. away: a robot already too
        close to an edge or another robot may take it if it never gets closer and ends clear."""
        if self.edge_rule != "fixed":
            if not self.edge_ok(r, tr, getattr(tr, "v", None), away=(away or edge_away), stops=False):
                return False
        else:
            if not self.invariant_ok(r, tr):
                return False
            x0, y0, x1, y1 = self.rect
            bub = r.bubble if r.lost else 0.0
            xs, ys, ms = tr.x[:-1], tr.y[:-1], tr.m[:-1]
            if len(xs):
                ed = edge_dist_v(self.rect, xs, ys) - (ms - PLAN_M0)
                need = LOOP_EDGE + bub
                if away or edge_away:
                    need = np.minimum(need, ed[0] - 0.5)
                if (ed < need).any():
                    return False
            c = tr.loop_c
            if c is not None and edge_dist(self.rect, *c) - (tr.m[-1] - LOOP_R - PLAN_M0) < LOOP_R + LOOP_EDGE + bub:
                return False
        if not self.keepout_ok(r, tr):
            return False
        for o in self.others(r):
            if o.name in ignore:
                continue
            need = PLAN_CC - gap_relax
            if not away:
                if clearance(tr, o.traj, now=self.t) < need:
                    return False
                continue
            prof = clearance_profile(tr, o.traj, now=self.t)
            if prof[-1] < need or (prof < np.minimum(need, prof[0] - 0.5)).any():
                return False
        return True

    def sides(self, x, y, h):
        """Loop sides, the one whose loop centre is further from the edges first."""
        a, b = loop_centre(x, y, h, 1), loop_centre(x, y, h, -1)
        return (1, -1) if edge_dist(self.rect, *a) >= edge_dist(self.rect, *b) else (-1, 1)

    def pick_curve(self, r, x, y, h, t0, v0=0.0, loop_side=None):
        """The next piece from (x, y, h) at t0: the menu (least-used first), then off-menu (gentler / shorter), then
        the loop in its own reserved disc (loop_side: the side reserved now), then an escape that never gets closer.
        Returns (kind, cell, k, L, v, side, traj) or None."""
        m0 = self.margin0(r)
        grow = BLIND_PER_CM if r.lost is not None else 0.0
        find_me = r.lost is not None and (r.bubble > BUBBLE_CAP if BLIND_RULE == "cap" else
                                          self.blind_stop(r, (x, y), max(v0, 5.0)))
        # starting closer to an edge than a curve may go (after a stop there): curves that never get closer to it
        ea = EDGE_AWAY and edge_dist(self.rect, x, y) < (LOOP_EDGE if self.edge_rule == "fixed" else
                                                          self.turn_need(r) - (r.bubble if r.lost else 0.0)) + 3.0 + \
            (r.bubble if r.lost else 0.0)

        def tr_for(k, L, v, side):
            return curve_traj(t0, [(x, y, h, k, L, v)], side, m0=m0, bubble_grow=grow)
        if not find_me:
            free = self.rng.random() < FREE_SHARE
            for cell in self.cmenu.order(r.name, free=free):
                k, L, v = self.cmenu.draw(cell)
                ex, ey, eh = arc_at(x, y, h, k, L)
                for side in self.sides(float(ex), float(ey), float(eh)):
                    tr = tr_for(k, L, v, side)
                    if self.curve_ok(r, tr, edge_away=ea):
                        return ("free" if free else "menu"), cell, k, L, v, side, tr
            for L in (14.0, 8.0):
                ks = [kk * sg for kk in (1 / 40.0, 1 / 20.0, 1 / 12.0) for sg in (1, -1)]
                self.rng.shuffle(ks)
                for k in [0.0] + ks:
                    ex, ey, eh = arc_at(x, y, h, k, L)
                    for side in self.sides(float(ex), float(ey), float(eh)):
                        tr = tr_for(k, L, 5.0, side)
                        if self.curve_ok(r, tr, edge_away=ea):
                            self.fallback(r, 1, "off-menu curve", "no menu curve fits")
                            return "off-menu", None, k, L, 5.0, side, tr
        # the loop: round its own reserved disc (the last piece's end reserved it), a quarter circle at a time
        sides = (loop_side,) if loop_side else self.sides(x, y, h)
        for side in sides:
            k = side / LOOP_R
            L = math.pi * LOOP_R / 2
            v = min(5.0 if not find_me else 4.0, v_cap(k))
            tr = tr_for(k, L, v, side)
            if loop_side or self.curve_ok(r, tr, gap_relax=PLAN_CC - REFLEX_CC, away=True):
                self.counts["loops"] += 1
                self.fallback(r, 4 if not find_me else 6, "find-me loop" if find_me else "loop",
                              "lost, bubble %.1f cm: circles in its own disc until seen" % r.bubble if find_me else
                              "nothing else fits: circles in its own reserved disc (never stops)")
                return ("loop" if not find_me else "find-me loop"), None, k, L, v, side, tr
        # escape: any curve that never brings it closer to an edge or anyone and ends clear
        for k in (0.0, 1 / 25.0, -1 / 25.0, 1 / 12.0, -1 / 12.0, 1 / 8.0, -1 / 8.0):
            for L in (20.0, 10.0, 5.0):
                ex, ey, eh = arc_at(x, y, h, k, L)
                for side in self.sides(float(ex), float(ey), float(eh)):
                    tr = tr_for(k, L, 4.0, side)
                    if self.curve_ok(r, tr, gap_relax=PLAN_CC - REFLEX_CC, away=True):
                        self.fallback(r, 2, "escape curve", "nothing else fits: a curve that only moves away")
                        return "escape", None, k, L, 4.0, side, tr
        return None

    def send_piece(self, r, kind, cell, k, L, v, side, x, y, h, t0):
        sid = self.send_step(r, "fs %%d R %.2f %.2f %.2f %.5f %.2f %.2f -" % (x, y, h, k, L, v))
        ex, ey, eh = arc_at(x, y, h, k, L)
        if cell is not None:
            self.cmenu.use(cell, r.name)
        self.counts["pieces"] += 1
        r.n_moves += 1
        return dict(kind=kind, cell=cell, turn=0.0, L=L, v=v, t0=self.t, p0=(x, y), h0=h, h1=float(eh),
                    end=(float(ex), float(ey)), phase="queued", n=r.n_moves, drive_id=sid, turn_id=None, t_drive=None,
                    adj_scale=1.0, rev=False, curve=True, k=k, side=side, x=x, y=y, h=h, t_plan=t0)

    def start_curve(self, r):
        """A curve robot with no piece running (the start, a coast after the last piece, a dodge, after a stop):
        the next piece from where it will be in a moment."""
        p, h = self.pos(r), self.heading(r)
        if getattr(r, "tilted", False) and self.tilt_recovery:
            r.back_off = None
            r.coast = None
            return self.recover_step(r)            # (e08, A3) the recovery tries, never towards the drop
        if r.back_off is not None:
            # its own front check STOPPED it (no dodge): back straight off first, as in stop-turn (a speed-zero moment)
            bo_, r.back_off = r.back_off, None
            res = self.track_backout(r) if (EDGE_BACKOUT and bo_.get("edge")) else None   # (e16, 7) its own track
            if res is None:
                res = self.back_move(r)
            if res is None and getattr(r, "tilted", False):
                res = self.back_move(r, inward=True)       # (11-05h) tilted at the edge: any back-off that gains room
            if res is not None:
                self.counts["backoffs"] += 1
                self.fallback(r, 0, "back off", "its own front stop: backs away before anything else")
                self.stopturn_move(r, res)
                self.why_zero(r, "back off after its own front stop")
                r.tilted = False
                return
            if getattr(r, "tilted", False):
                # (11-05h) tilted and no room to back: stand (never forward), tried again
                r.back_off = dict(t=self.t, h=self.heading(r))
                r.busy_until = self.t + 1.0
                if not r.stopped_hard:
                    self.trophy_break(r, "tilted and no room to back away: stands until it can back off")
                return
        co = r.coast if (r.coast and self.t - r.coast["t"] < COAST_S) else None
        lead = 0.3
        if r.__dict__.get("probe_ret"):
            res_r = self.probe_return(r)               # (e10, C2) back from a probe
            if res_r is not None:
                r.coast = None
                self.stopturn_move(r, res_r)
                return
        if self.probe_ahead:
            res_p = self.try_probe(r)                  # (e10, C2) the probe-ahead setting (off by default): curve
            if res_p is not None:                      # robots rarely stand, so it is tried between pieces too
                r.coast = None
                self.stopturn_move(r, res_p)
                return
        if co:
            ex, ey, eh = arc_at(p[0], p[1], h, co["k"], co["v"] * lead)
            x, y, h1, v0 = float(ex), float(ey), float(eh), co["v"]
        else:
            x, y, h1, v0 = p[0], p[1], h, 0.0
            lead = 0.2
        res = self.pick_curve(r, x, y, h1, self.t + lead, v0)
        if res is None:
            if co is not None:
                # nothing fits from the coast: a loop the way it is curving (it is rolling; stopping is worse)
                side = 1 if co["k"] >= 0 else -1
                for sd in (side, -side):
                    k, L = sd / LOOP_R, math.pi * LOOP_R / 2
                    tr = curve_traj(self.t + lead, [(x, y, h1, k, L, 4.0)], sd, m0=self.margin0(r))
                    if self.curve_ok(r, tr, gap_relax=PLAN_CC - HARD_CC, away=True):
                        res = ("loop", None, k, L, min(4.0, v_cap(k)), sd, tr)
                        self.fallback(r, 5, "loop (forced)", "nothing fits from the coast: loops on (never closer)")
                        break
            if res is None:
                # standing and no curve fits (an edge, a trap at the start): the stop-turn ladder (reverse, edge turn)
                r.coast = None
                self.why_zero(r, "no curve fits from a stand: stop-turn rung")
                res2 = self.pick(r)
                if res2 is None and EDGE_BACKOUT:
                    res2 = self.track_backout(r)       # (e16, 7: home 08:48) back along its own track, then re-plan
                if res2 is None:
                    r.busy_until = self.t + 1.0
                    if not r.stopped_hard:
                        self.trophy_break(r, "nothing fits from a spot too near the edge to turn")
                    return
                self.stopturn_move(r, res2)
                return
        kind, cell, k, L, v, side, tr = res
        if r.stopped_hard and (r.adj is None or r.adj["kind"] != "stop"):
            self.close_break(r)
        r.coast = None
        mv = self.send_piece(r, kind, cell, k, L, v, side, x, y, h1, self.t + lead)
        r.move, r.queue, r.mode = mv, [], "move"
        r.hold_n = 0
        self.retraj_curve(r)
        self.pump(r)

    # -- (e10, the 7 Nov run 2 incident) the queue watch ---------------------------------------------------------
    def course(self, r):
        """The way r really goes: the camera's track over the last COURSE_S against its own pose's track over the same
        moments (both bend alike on a curve, so the difference is its own heading's error). Returns (deg, err_deg,
        source): its own heading turned by the error ("camera"), else the filter's heading, else its own (err None)."""
        if r.lost is None and r.cam is not None and self.fresh(r):
            h = [q for q in r.cam_hist if q[0] >= self.t - COURSE_S]
            if len(h) >= 4 and math.dist(h[0][1:], h[-1][1:]) >= COURSE_MIN_CM:
                a = ((h[0][1] + h[1][1]) / 2, (h[0][2] + h[1][2]) / 2, (h[0][0] + h[1][0]) / 2)
                b = ((h[-1][1] + h[-2][1]) / 2, (h[-1][2] + h[-2][2]) / 2, (h[-1][0] + h[-2][0]) / 2)
                oa, ob = self.own_at(r, a[2]), self.own_at(r, b[2])
                dc = (b[0] - a[0], b[1] - a[1])
                if oa is not None and ob is not None:
                    do = (ob[0] - oa[0], ob[1] - oa[1])
                    if math.hypot(*do) >= 0.6 * math.hypot(*dc) and math.hypot(*dc) >= 0.6 * COURSE_MIN_CM:
                        err = wrap180(math.degrees(math.atan2(dc[1], dc[0]) - math.atan2(do[1], do[0])))
                        return wrap180(self.heading(r) + err), err, "camera"
        if self.ekf_live(r):
            q = r.fp.pose(self.t)
            if q is not None:
                return float(q[2]), None, "filter"
        return self.heading(r), None, "own"

    def off_track(self, mv, x, y, v_now, t_at):
        """How far (x, y) is ACROSS the running piece's path, at t_at (cm): a straight's side distance, a curve's
        distance from its circle (along the path does not count: the firmware ends the piece by its length)."""
        done = min(mv["L"], max(0.0, (t_at - mv["t_drive"]) * v_now))
        k = mv["k"]
        if abs(k) < 1e-4:
            hr = math.radians(mv["h"])
            return abs(math.cos(hr) * (y - mv["y"]) - math.sin(hr) * (x - mv["x"]))
        hr, R = math.radians(mv["h"]), 1.0 / abs(k)
        sg = 1.0 if k > 0 else -1.0
        cx, cy = mv["x"] - math.sin(hr) * sg * R, mv["y"] + math.cos(hr) * sg * R
        ex, ey, _ = arc_at(mv["x"], mv["y"], mv["h"], k, done)
        return min(abs(math.hypot(x - cx, y - cy) - R) + max(0.0, math.dist((x, y), (float(ex), float(ey))) - 2 * R),
                   math.dist((x, y), (float(ex), float(ey))))

    def real_segs(self, r, p, c):
        """The running piece's rest and its queued pieces, chained from where r really is (p) the way it really goes
        (c): where they would take it (curve_segs chains the queued ones from their PLANNED starts)."""
        mv = r.move
        sc = r.scale if r.scale > 0 else 1.0
        v = mv["v"] * sc
        done = (self.t - mv["t_drive"]) * max(0.5, v)
        rem = max(0.0, mv["L"] - done)
        x, y, h = p[0], p[1], c
        segs = []
        for k, L, vv in [(mv["k"], rem, v)] + [(q["k"], q["L"], q["v"] * sc) for q in r.queue]:
            if L > 0.2:
                segs.append((x, y, h, k, L, vv))
                ex, ey, eh = arc_at(x, y, h, k, L)
                x, y, h = float(ex), float(ey), float(eh)
        return segs, (r.queue[-1]["side"] if r.queue else mv["side"])

    def queue_watch(self, r):
        """(e10, run 2 incident a, c) Every tick while a curve piece runs: is r where the piece says (the fused
        position, and the camera's own fix at its time: ACROSS the path, OFFPLAN_CM for OFFPLAN_S)? Do the piece's rest
        and the pieces queued behind it keep the edge rule from where it really is, the way it really goes? If not,
        the queued pieces are dropped and the piece re-aimed from the fused position (farc), or, with its own heading
        off the camera's course or too little of the piece left, it is stopped (its heading re-set first) and the next
        piece planned from where it stands. A long straight is checked all along against the camera this way."""
        mv = r.move
        if not self.queue_watch_on or mv is None or not mv.get("curve") or mv.get("t_drive") is None or \
                mv["phase"] != "drive" or r.lost is not None or r.silent or r.mode != "move":
            return
        if str(mv["kind"]).startswith(("probe", "recover", "unsure", "shuffle")):
            return
        p = self.pos(r)
        if p is None:
            return
        v_now = max(0.5, mv["v"] * (r.scale if r.scale > 0 else 1.0))
        d = self.off_track(mv, p[0], p[1], v_now, self.t)
        if r.cam is not None and self.fresh(r) and r.cam[0] >= mv["t_drive"]:
            d = max(d, self.off_track(mv, r.cam[1], r.cam[2], v_now, r.cam[0]))
        why = None
        if d > OFFPLAN_CM:
            mv.setdefault("qw_off_t", self.t)
            if self.t - mv["qw_off_t"] >= OFFPLAN_S:
                why = "off its piece by %.1f cm (the fused position)" % d
        else:
            mv.pop("qw_off_t", None)
        c, err, src = self.course(r)
        # its own heading off the camera's course by COURSE_ERR_DEG for COURSE_ERR_S: the rest of the piece and the
        # queued ones are checked from where it really is, the way it really goes
        if err is not None and abs(err) >= COURSE_ERR_DEG:
            mv.setdefault("qw_err_t", self.t)
        else:
            mv.pop("qw_err_t", None)
        if why is None and mv.get("qw_err_t") is not None and self.t - mv["qw_err_t"] >= COURSE_ERR_S:
            segs, side = self.real_segs(r, p, c)
            if segs:
                tr = curve_traj(self.t, segs, side, m0=self.margin0(r))
                n_ref = self.counts.get("edge_refused", 0)
                ok = self.edge_ok(r, tr, getattr(tr, "v", None), away=True, stops=False)
                self.counts["edge_refused"] = n_ref
                if not ok:
                    why = "its piece%s from where it really is (going %.0f deg) would break the edge rule" % (
                        " and the %d queued" % len(r.queue) if r.queue else "", c)
        if why is None:
            return
        if self.t - mv.get("qw_t", -1e9) < 1.0:
            return
        mv["qw_t"] = self.t
        self.counts["queue_watch"] = self.counts.get("queue_watch", 0) + 1
        nq = len(r.queue)
        for q in r.queue:
            st = r.steps.get(q["drive_id"])
            if st and st["status"] in ("queued_local", "sent", "acked", "started"):
                st["status"] = "dropped"
        r.queue = []
        own_h = self.heading(r)
        herr = abs(err) if err is not None else 0.0
        left_s = (mv["L"] - (self.t - mv["t_drive"]) * v_now) / v_now
        self.log("pose", dict(t=round(self.t, 2), robot=r.name, what="queue watch: " + why, d_cm=round(d, 2),
                              unc_cm=round(self.unc(r), 2)))
        if herr > COURSE_RESET_DEG:
            # its own heading is wrong (the piece runs in its own frame): set it from the camera's course, stop, and
            # plan the next piece from where it stands
            self.counts["queue_heading_resets"] = self.counts.get("queue_heading_resets", 0) + 1
            self.out(r, "fpose %.2f %.2f %.2f 1.0 3.0" % (p[0], p[1], c))
            if HEADING_CONFIRM:     # (e17, 2) the next move waits for a camera-confirmed heading
                r.hd_confirm = dict(t=self.t, why="off-plan heading re-set (%.0f deg)" % herr, tries=0, stage=None)
            self.event(r, "off plan", "%s; %d queued dropped; its own heading %.0f deg, the camera sees it go %.0f: "
                       "heading re-set, stopped, re-planned from where it is" % (why, nq, own_h, c))
            self.halt(r, "off plan: heading re-set")
            self.end_move(r)
            return
        res = self.pick_curve(r, p[0], p[1], c, self.t) if left_s > FARC_MIN_S else None
        if res is None:
            self.event(r, "off plan", "%s; %d queued dropped: stopped, re-planned from where it is" % (why, nq))
            self.halt(r, "off plan")
            self.end_move(r)
            return
        kind, cell, k, L, v, side, _ = res
        self.counts["reaimed"] = self.counts.get("reaimed", 0) + 1
        self.event(r, "off plan", "%s; %d queued dropped: re-aimed from the fused position (farc, no stop)" % (why, nq))
        self.control(r, "farc %.5f %.1f" % (k, L), "farc")
        ex, ey, eh = arc_at(p[0], p[1], c, k, L)
        mv.update(k=k, L=L, x=p[0], y=p[1], h=c, end=(float(ex), float(ey)), h1=float(eh), side=side, t_drive=self.t,
                  kind=mv["kind"] + "+offplan")
        mv.pop("qw_off_t", None)
        if cell is not None:
            self.cmenu.use(cell, r.name)
        self.retraj_curve(r)

    def predicted_end(self, r):
        """(e10) Where the running piece really ends: its rest from the fused position the way it really goes."""
        mv, p = r.move, self.pos(r)
        if mv is None or p is None or not self.queue_watch_on or not mv.get("curve") or mv.get("t_drive") is None:
            return mv["end"] if mv is not None else p
        c, err, _ = self.course(r)
        if err is None:
            c = self.heading(r)
        segs, _ = self.real_segs(r, p, c)
        if not segs:
            return p
        x, y, h, k, L, _ = segs[0]
        ex, ey, _ = arc_at(x, y, h, k, L)
        return float(ex), float(ey)

    def start_check(self, r):
        """(e10, run 2 incident d) Before its first move: a robot whose middle is not START_EDGE_CM inside the board by
        the camera (over the edge, or too near it to turn) is held with a plain message, checked again every 2 s,
        and starts once it is START_CLEAR_CM in (a person moved it). True: it may start."""
        if not self.start_edge or r.__dict__.get("start_ok"):
            return True
        if r.cam is None or not self.fresh(r):
            p = self.pos(r)
            if p is None:
                return False
        else:
            p = (r.cam[1], r.cam[2])
        # (e13) where the workspace is limited to the readable area, the start check judges the table's own edge; a
        # robot outside the readable area is the stray return's (it comes back), not a robot at an edge
        lim = self.ws is not None and getattr(self.ws, "limited", False)
        ed = (lambda x_, y_: self.ws.edge_dist(x_, y_, raw=True)) if lim else (lambda x_, y_: edge_dist(self.rect, x_, y_))
        e = ed(*p)
        h = r.cam[3] if (r.cam is not None and r.cam[3] is not None) else (self.heading(r) if r.own is not None else None)
        if h is None:
            ec = e - HALF                    # heading unknown: the body's circle
        else:
            hr = math.radians(h)
            ec = min(ed(p[0] + a * math.cos(hr) - b * math.sin(hr),
                        p[1] + a * math.sin(hr) + b * math.cos(hr))
                     for a in (-BODY / 2, BODY / 2) for b in (-BODY / 2, BODY / 2))
        held = r.__dict__.get("start_held")
        rec = r.__dict__.get("start_recovering")
        if ec >= (START_CLEAR_CM if (held or rec) else START_CORNER_CM) and not (rec and getattr(r, "tilted", False)):
            r.start_ok = True
            if held:
                self.event(r, "start check", "now %.0f cm inside the board's edge: it starts" % e)
                self.close_break(r)
            elif rec:
                self.counts["start_freed"] = self.counts.get("start_freed", 0) + 1
                self.event(r, "start check", "freed itself: now %.0f cm inside the board's edge, it starts" % e)
            return True
        if self.start_mode == "recover" and self.tilt_recovery and not held:
            # (Nick 7 Nov) first it tries to free itself, as a robot that tilts at an edge during a run does
            # (recover_step: straight back when backing is inward by the camera and its own pose, then a reverse to
            # each side, then a short inward pivot); recover_step itself holds and asks a person when nothing safe
            # remains, and tries again later
            if not rec:
                r.start_recovering = self.t
                self.counts["start_recover"] = self.counts.get("start_recover", 0) + 1
                where = "its middle is over the edge" if e < 0 else ("part of its body is over the edge" if ec < 0
                                                                     else "it is right at the edge")
                self.event(r, "start check", "%s starts at the board's edge: %s (the camera puts its middle %.1f cm "
                           "from the edge). It tries to free itself first (backing straight inward if the camera and "
                           "its own pose agree that is inward); a person is asked only if that fails." % (
                               r.name, where, e))
            dev = self.tilt_dev(r)
            if dev is not None and dev > SR.tilt_limits()["stand_tilt_deg"]:
                return True                  # it reads a tilt: hung; its tilt watch starts the same recovery as in a run
            if not getattr(r, "tilted", False):
                if r.__dict__.get("start_tried"):
                    # its recovery said it backed off but it is still at the edge (a hung robot's wheels turn, its
                    # body stays): from here the run's own edge rules take it, as for a robot hung during a run (the
                    # edge trap's reverse, the corner shuffle, the unsure step: none of them ever goes outward)
                    r.start_ok = True
                    self.event(r, "start check", "its recovery tries are done and it is still %.0f cm from the edge: "
                               "the run's own edge rules take it from here" % e)
                    return True
                r.start_tried = True
                r.tilted = True              # the recovery tries run through start_move / start_curve
                r.back_off = dict(t=self.t, h=self.heading(r))
            return True
        if not held:
            r.start_held = self.t
            self.counts["start_held"] = self.counts.get("start_held", 0) + 1
            where = "its middle is over the edge" if e < 0 else ("part of its body is over the edge" if ec < 0 else
                                                                 "it is right at the edge")
            msg = ("%s starts at the board's edge: %s (the camera puts its middle %.1f cm from the edge). It will not "
                   "move. Please move it a hand's width (10 cm) onto the board; it starts by itself once it is there." % (
                       r.name, where, e))
            self.event(r, "start check", msg)
            self.trophy_break(r, "starts at the edge: held until moved in")
            if self.attended:
                self.ask(r, msg, ("I moved it in", ))
            else:
                self.want_picture(r, "starts at the edge")
        r.busy_until = self.t + 2.0
        return False

    def plan_next(self, r):
        """While a piece runs: the next one from its planned end, queued on the robot so they run into each other."""
        mv = r.move
        if r.__dict__.get("late") is not None or r.__dict__.get("hd_confirm") is not None:
            return                  # (e17, 2) slowed / waiting on late fixes, or its heading in doubt: nothing queued
        v_now = max(0.5, mv["v"] * (r.scale if r.scale > 0 else 1.0))
        if self.pose_rule != "wait2s" and mv.get("t_drive") is not None:
            # (e08, A1) the next piece is chained from this one's PLANNED end only while the robot is where the plan
            # says (the fused position: the camera, latency-compensated); off it by more than SURE_TOL_CM, nothing is
            # queued: the piece runs out (coast) and the next one starts from where the robot really is
            done = (self.t - mv["t_drive"]) * v_now
            ex, ey, _ = arc_at(mv["x"], mv["y"], mv["h"], mv["k"], min(mv["L"], max(0.0, done)))
            p = self.pos(r)
            if p is not None and math.hypot(p[0] - float(ex), p[1] - float(ey)) > SR.SURE_TOL_CM + PLAN_M0:
                if not mv.get("off_plan"):
                    mv["off_plan"] = True
                    self.counts["off_plan"] = self.counts.get("off_plan", 0) + 1
                    self.log("pose", dict(t=round(self.t, 2), robot=r.name, what="off plan: next piece from the "
                                          "fused position", d_cm=round(math.hypot(p[0] - float(ex), p[1] - float(ey)), 2),
                                          unc_cm=round(self.unc(r), 2)))
                return
        if mv.get("t_drive") is not None:
            t_end = mv["t_drive"] + mv["L"] / v_now
        else:
            t_end = self.t + 0.3 + mv["L"] / v_now
        t_end = max(t_end, self.t + 0.2)
        last = r.queue[-1] if r.queue else mv
        x, y, h = last["end"][0], last["end"][1], last["h1"]
        res = self.pick_curve(r, x, y, h, t_end, mv["v"], loop_side=None)
        if res is None:
            side = last["side"]
            k, L = side / LOOP_R, math.pi * LOOP_R / 2
            res = ("loop", None, k, L, min(5.0, v_cap(k)), side, None)
            self.counts["loops"] += 1
            self.fallback(r, 4, "loop", "nothing fits: circles in its own reserved disc (never stops)")
        kind, cell, k, L, v, side, _ = res
        nx = self.send_piece(r, kind, cell, k, L, v, side, x, y, h, t_end)
        r.queue.append(nx)
        self.retraj_curve(r)
        self.pump(r)

    def curve_segs(self, r, scale=None):
        """The rest of r's curve plan as segments from where it is now: (x, y, h, k, L, v) ..., and its loop side."""
        p, h = self.pos(r), self.heading(r)
        mv = r.move
        sc = r.scale if scale is None else scale
        if mv is None:
            co = r.coast
            if co and self.t - co["t"] < COAST_S:
                return [(p[0], p[1], h, co["k"], co["v"] * max(0.0, COAST_S - (self.t - co["t"])), co["v"])], \
                    (1 if co["k"] >= 0 else -1)
            return [], 1
        v = mv["v"] * sc
        done = (self.t - mv["t_drive"]) * v if mv.get("t_drive") is not None else 0.0
        rem = max(0.0, mv["L"] - done)
        segs = [(p[0], p[1], h, mv["k"], rem, v)] if rem > 0.2 else []
        side = mv["side"]
        for q in r.queue:
            segs.append((q["x"], q["y"], q["h"], q["k"], q["L"], q["v"] * sc))
            side = q["side"]
        return segs, side

    def retraj_curve(self, r):
        p = self.pos(r)
        segs, side = self.curve_segs(r)
        if not segs:
            r.traj = Traj.stand(self.t, *p, m=self.margin0(r))
            return
        r.traj = curve_traj(self.t, segs, side, m0=self.margin0(r),
                            bubble_grow=BLIND_PER_CM if r.lost is not None else 0.0)

    def curve_pts(self, r, scale=None, bend=None, H=REFLEX_H_S, dt=REFLEX_STEP_S):
        """Where a curve robot will be over the next H s (positions every dt): its pieces in order, then its loop;
        bend = (k, L): the running piece bent now (farc) to curvature k for L cm, then its loop that way."""
        ts = np.arange(0.0, H + 1e-9, dt)
        p, h = self.pos(r), self.heading(r)
        mv = r.move
        coasting = bool(r.coast and self.t - r.coast["t"] < COAST_S)
        if (mv is None and not coasting) or (mv is not None and mv["phase"] != "drive" and not coasting) or \
                r.lost_hold:
            return np.tile(np.array(p, float), (len(ts), 1)), 0.0
        sc = r.scale if scale is None else scale
        if mv is None or mv["phase"] != "drive":
            v = r.coast["v"]
            sched = [(r.coast["k"], v * max(0.0, COAST_S - (self.t - r.coast["t"])))]
            side = 1 if r.coast["k"] >= 0 else -1
        else:
            v = mv["v"] * sc
            if bend is not None:
                sched = [(bend[0], bend[1])]
                side = 1 if bend[0] >= 0 else -1
            else:
                done = (self.t - mv["t_drive"]) * v if mv.get("t_drive") is not None else 0.0
                sched = [(mv["k"], max(0.0, mv["L"] - done))] + [(q["k"], q["L"]) for q in r.queue]
                side = (r.queue[-1] if r.queue else mv)["side"]
        sched.append((side / LOOP_R, 1e9))
        s_all = v * ts
        out = np.empty((len(ts), 2))
        x, y, hh, s0 = p[0], p[1], h, 0.0
        j = 0
        for k, L in sched:
            sel = (s_all >= s0 - 1e-9) & (s_all <= s0 + L + 1e-9)
            idx = np.nonzero(sel)[0]
            if len(idx):
                xs, ys, _ = arc_at(x, y, hh, k, s_all[idx] - s0)
                out[idx, 0], out[idx, 1] = xs, ys
            ex, ey, eh = arc_at(x, y, hh, k, min(L, s_all[-1] - s0 + 1.0) if L > 1e8 else L)
            x, y, hh = float(ex), float(ey), float(eh)
            s0 += L
            j += 1
            if s0 > s_all[-1]:
                break
        return out, v

    def try_adjust_curve(self, a, other, pred_before):
        """A curve robot's avoidance: slower (never under v_min) or faster, then a bend (farc, smallest first), then a
        bend at the slowest; the first that keeps every predicted gap (and, for a bend, the edges, the keep-outs and the
        others' reservations at the reflex's gap) is applied. Nothing keeps it: the bend that leaves the most room (no
        stop) unless the circles would touch within HARD_T_S."""
        mv = a.move
        if mv is None:
            return False
        v = mv["v"]
        other_moves = self.moving(other)
        cands = []
        if other_moves:
            for sfac in SPEED_OPTIONS:
                sc = max(sfac, self.v_min / max(v, 0.1))
                if v * sc > v_cap(mv["k"]) or abs(sc - a.scale) < 0.01 or (sc < 1 and v * a.scale <= self.v_min):
                    continue
                cands.append(("speed", sc, None))
        for kk in AVOID_K:
            cands.append(("bend", a.scale, (kk, 25.0)))
        for kk in AVOID_K:
            cands.append(("bend", max(self.v_min / max(v, 0.1), 0.0) if v > self.v_min else a.scale, (kk, 25.0)))
        best = None
        p, h = self.pos(a), self.heading(a)
        for kind, sc, bend in cands:
            pts, vv = self.curve_pts(a, scale=sc, bend=bend)
            g, o2, tm = self.worst_for(a, pts)
            ok_geo = True
            if bend is not None:
                side = 1 if bend[0] >= 0 else -1
                tr = curve_traj(self.t, [(p[0], p[1], h, bend[0], bend[1], max(0.5, v * sc))], side,
                                m0=self.margin0(a))
                ok_geo = self.curve_ok(a, tr, gap_relax=PLAN_CC - REFLEX_CC, away=True)
            if ok_geo and (best is None or g > best[0]):
                best = (g, kind, sc, bend, o2)
            if g >= 0.5 and ok_geo:
                self.apply_curve(a, other, kind, sc, bend, pred_before, g + self.need(a, o2) if o2 else 99.0)
                return True
        if best is not None:
            g, kind, sc, bend, o2 = best
            hard = g + (self.need(a, o2) if o2 else REFLEX_CC) < HARD_CC
            if not hard:
                self.apply_curve(a, other, kind + " (best)", sc, bend, pred_before,
                                 g + self.need(a, o2) if o2 else 99.0)
                return True
        return False

    def apply_curve(self, a, other, kind, scale, bend, pred_before, pred_after):
        self.ev_id += 1
        ev = dict(id=self.ev_id, t=self.t, actor=a.name, other=other.name, kind=kind, scale=scale,
                  steer=None, pred_before=pred_before, pred_after=pred_after,
                  method="farc" if bend else "fspd", clear_since=None, min_cc=float("inf"), bend=bend)
        a.adj = ev
        key = "bend" if bend else "speed"
        self.counts["reflex"][key] = self.counts["reflex"].get(key, 0) + 1
        if abs(scale - a.scale) > 0.005:
            self.control(a, "fspd %.2f" % scale, "fspd")
            a.scale = scale
        if bend is not None:
            p, h = self.pos(a), self.heading(a)
            self.control(a, "farc %.5f %.1f" % bend, "farc")
            mv = a.move
            for q in a.queue:
                st = a.steps.get(q["drive_id"])
                if st:
                    st["status"] = "dropped"
            a.queue = []
            ex, ey, eh = arc_at(p[0], p[1], h, bend[0], bend[1])
            mv.update(k=bend[0], L=bend[1], x=p[0], y=p[1], h=h, end=(float(ex), float(ey)), h1=float(eh),
                      side=1 if bend[0] >= 0 else -1, t_drive=self.t, kind=mv["kind"] + "+bend")
        self.retraj_curve(a)

    def arc_fit(self, r, mv):
        """(11-05d) The curves' calibration, discovered from the run (CLAUDE.md: discover, don't pre-map): the camera's
        spots along a finished curve piece fitted with a circle (its last 80 %: the start bends from the last piece);
        k ratio = the fitted curvature / the one asked, length ratio = the camera's arc (angle swept x radius) / the
        length asked. Logged (arcfit.csv); medians are saved on the robot when it parks (arc_save)."""
        pts, r.arc_pts = r.arc_pts, []
        k, L = mv["k"], mv["L"]
        if "bend" in mv["kind"]:
            return
        f = fit_arc_points(pts, k, L)
        if f is None:
            return
        kr, dr, R = f
        r.arc_fits.append((kr, dr))
        self.log("arcfit", dict(t=round(self.t, 2), robot=r.name, k=round(k, 5), L=round(L, 1), v=round(mv["v"], 2),
                                R_fit=round(R, 2), k_ratio=round(kr, 4), length_ratio=round(dr, 4), n=len(pts)))

    def arc_result(self, r):
        """(median k ratio, its 95 % half-width, median length ratio, its half-width, n) or None."""
        return arc_summary(r.arc_fits)

    def adj_watch_curve(self, a, ev):
        o = self.R[ev["other"]]
        if ev["kind"] == "stop" and self.t - ev["t"] >= 0.6:
            pts, _ = self.curve_pts(a, scale=1.0)
            if self.worst_for(a, pts)[0] < 2.0:
                # a full stop lasts only until a new piece goes away (as in stop-turn): from the stand
                self.halt(a, "after the hard stop: a new piece away from %s" % o.name)
                self.end_move(a)
                a.busy_until = self.t
                return
        if ev.get("bend") is not None and abs(ev["scale"] - 1.0) < 0.01:
            self.close_adj(a, "bent")
            return
        pts, _ = self.curve_pts(a, scale=1.0)
        g, _, _ = self.worst_for(a, pts)
        if g >= 2.0:
            if ev["clear_since"] is None:
                ev["clear_since"] = self.t
            elif self.t - ev["clear_since"] >= CLEAR_HOLD_S:
                a.scale = 1.0
                self.control(a, "fspd 1.00", "fspd")
                self.retraj_curve(a)
                self.close_adj(a, "clear")
        else:
            ev["clear_since"] = None
            if self.t - ev["t"] > 0.4:
                pts, _ = self.curve_pts(a)
                gc, oc, tm = self.worst_for(a, pts)
                if gc < 0 and oc is not None:
                    pb = gc + self.need(a, oc)
                    self.close_adj(a, "escalated")
                    if not self.try_adjust_curve(a, oc, pb) and pb < HARD_CC and tm <= HARD_T_S:
                        self.curve_stop(a, oc, pb)

    def curve_stop(self, a, other, pred):
        """The last resort: nothing keeps the circles apart within HARD_T_S: fspd 0 (a speed-zero moment, logged)."""
        self.ev_id += 1
        ev = dict(id=self.ev_id, t=self.t, actor=a.name, other=other.name, kind="stop", scale=0.0, steer=None,
                  pred_before=pred, pred_after=pred, method="fspd", clear_since=None, min_cc=float("inf"), bend=None)
        a.adj = ev
        self.counts["reflex"]["stop"] = self.counts["reflex"].get("stop", 0) + 1
        self.trophy_break(a, "reflex hard stop: %s predicted %.1f cm from %s in %.1f s" % (a.name, pred, other.name,
                                                                                           HARD_T_S))
        self.why_zero(a, "reflex hard stop")
        self.control(a, "fspd 0.00", "fspd")
        a.scale = 0.0
        self.retraj_curve(a)

    def moving(self, o):
        if o.curve and (o.move is None or o.move.get("curve")):
            return o.move is not None and o.move["phase"] == "drive" and o.scale > 0 or bool(
                o.coast and self.t - o.coast["t"] < COAST_S)
        return self.motion(o)[2] > 0

    def stopturn_move(self, r, res):
        """A stop-turn move (T + D steps) for a curve robot that cannot curve from where it stands."""
        kind, cell, turn, L, v, tr = res
        p, h = self.pos(r), self.heading(r)
        h1 = wrap180(h + turn)
        rev = kind in ("back off", "reverse", "shuffle back", "probe return", "stray return back")
        mv = dict(kind=kind, cell=None, turn=turn, L=L, v=v, t0=self.t, p0=p, h0=h, h1=h1, end=tr.end,
                  phase="turn", n=r.n_moves + 1, drive_id=None, turn_id=None, t_drive=None, adj_scale=1.0, rev=rev,
                  track_back=bool(getattr(tr, "track_back", False)))
        if abs(turn) >= 3.0:
            mv["turn_id"] = self.send_step(r, "fs %%d T %.2f -" % h1)
        if L > 0:
            mv["drive_id"] = self.send_step(r, "fs %%d D %.2f %.2f %.2f %.2f %.2f %.2f %d -" % (
                p[0], p[1], tr.end[0], tr.end[1], h1, v, 1 if rev else 0))
        if self.empty_move(r, mv, p, kind):
            return
        if kind == "probe":
            r.probe_active = dict(cells=r.__dict__.pop("probe_cells", []), p0=p, t=self.t)
        r.move, r.traj, r.mode, r.queue = mv, tr, "move", []
        r.n_moves += 1
        self.why_zero(r, "%s (stop-turn rung: no curve fits from there)" % kind)
        self.pump(r)

    def curve_step_over(self, r, sid, state, info):
        """A curve piece ended: done -> the queued piece runs on (no stop) or the robot coasts and the next piece is
        planned at once; dodged -> a keep-out (felt) and the next piece from the dodge; anything else stopped it."""
        mv = r.move
        if mv is None:
            return
        if sid != mv.get("drive_id"):
            if state in ("failed", "dropped"):
                r.queue = [q for q in r.queue if q["drive_id"] != sid]
            return
        p = self.pos(r)
        self.log("moves", dict(t=round(mv["t0"], 2), robot=r.name, n=mv["n"], kind=mv["kind"],
                               cell=CurveMenu.label(mv["cell"]) if mv["cell"] else "",
                               turn_deg=round(math.degrees(mv["k"] * mv["L"]), 1), length_cm=round(mv["L"], 1),
                               speed=round(mv["v"], 2), x0=round(mv["x"], 1), y0=round(mv["y"], 1),
                               x1=round(mv["end"][0], 1), y1=round(mv["end"][1], 1),
                               est_s=round(mv["L"] / max(0.5, mv["v"]), 2), dur_s=round(self.t - mv["t0"], 2),
                               result="done" if state == "done" else "%s: %s" % (state, info),
                               end_err_cm=round(math.dist(p, mv["end"]), 2) if p and state == "done" else "",
                               own_vs_cam_cm="", cam_x=round(r.cam[1], 1) if r.cam else "",
                               cam_y=round(r.cam[2], 1) if r.cam else "", style="curve",
                               curvature=round(mv["k"], 5)))
        if state == "done":
            self.arc_fit(r, mv)
            if r.queue:
                nx = r.queue.pop(0)
                nx["phase"], nx["t0"] = "drive", self.t
                if nx.get("t_drive") is None:
                    nx["t_drive"] = self.t
                r.move = nx
                self.retraj_curve(r)
                return
            r.coast = dict(t=self.t, k=mv["k"], v=max(2.0, mv["v"] * (r.scale if r.scale > 0 else 1.0)), dodge=False)
            self.why_zero(r, "coast ran out (the next piece was late)")
        elif state == "failed" and "dodged" in info:
            self.counts["dodges"] += 1
            self.event(r, "dodge", "its own front check: %s (no stop)" % info)
            k = DODGE_K if "left" in info else -DODGE_K
            self.felt(r, info.replace("dodged obstacle", "obstacle"))
            r.back_off = None
            r.coast = dict(t=self.t, k=k, v=4.0, dodge=True)
            self.why_zero(r, "dodge coast ran out")
        else:
            r.coast = None
            if "virtual border" in info:
                self.border_fire(r, info)
            if "tilted" in info:
                self.tilt_trip(r, info)
                self.why_zero(r, "its own tilt stop")
            elif "obstacle" in info or "plywood edge" in info:
                self.event(r, "front stop", "its own %s stopped the step: %s" % (
                    "front sensor" if "obstacle" in info else "edge guard", info))
                self.felt(r, info)
                self.why_zero(r, "its own %s" % ("front stop" if "obstacle" in info else "edge guard"))
            elif "watchdog" in info:
                self.event(r, "watchdog", "its watchdog stopped the step (nothing heard from the Pi)")
                self.why_zero(r, "watchdog (nothing heard from the Pi)")
            elif state == "failed":
                self.event(r, "step failed", info)
                self.why_zero(r, "step failed: %s" % info[:40])
        for q in r.queue:
            st = r.steps.get(q["drive_id"])
            if st and st["status"] in ("queued_local", "sent", "acked", "started"):
                st["status"] = "dropped"
        r.queue = []
        r.move = None
        r.controls = [c for c in r.controls if c["kind"] != "farc"]   # a bend for the piece that ended: not resent
        if r.adj is not None:
            self.close_adj(r, "move ended")
        if r.coast is None:
            r.scale = 1.0
        if r.mode == "move":
            r.mode = "idle"
        r.busy_until = self.t
        self.retraj_curve(r)

    # -- battery, parking ----------------------------------------------------------------------------------------
    def park_spot(self, r):
        p = self.pos(r)
        if p is None:
            return None
        x0, y0, x1, y1 = self.rect if self.ws is None else self.ws.board
        k = PARK_EDGE_CC if self.edge_rule == "fixed" else self.end_need(r, 5.0) + 3.0
        cands = []
        for i in range(48):
            f = i / 48.0
            per = 2 * (x1 - x0 - 2 * k) + 2 * (y1 - y0 - 2 * k)
            s = f * per
            w, hgt = x1 - x0 - 2 * k, y1 - y0 - 2 * k
            if s < w:
                q = (x0 + k + s, y0 + k)
            elif s < w + hgt:
                q = (x1 - k, y0 + k + s - w)
            elif s < 2 * w + hgt:
                q = (x1 - k - (s - w - hgt), y1 - k)
            else:
                q = (x0 + k, y1 - k - (s - 2 * w - hgt))
            cands.append(q)
        cands.sort(key=lambda q: math.dist(q, p))
        h = self.heading(r)
        for q in cands[:24]:
            L = math.dist(q, p)
            if L > 90:
                continue
            d = wrap180(math.degrees(math.atan2(q[1] - p[1], q[0] - p[0])) - h) if L > 1 else 0.0
            if L > 1 and not self.can_turn_here(r):
                continue
            tr = self.make(r, d, L, 5.0)
            if all(math.dist(q, self.pos(o)) > PLAN_CC + 8 for o in self.R.values()
                   if o is not r and self.pos(o) is not None) and self.feasible(r, tr):
                return d, L, tr
        return None

    def go_park(self, r, why):
        self.event(r, "parking", why)
        self.why_zero(r, "parking (battery)")
        sp = self.park_spot(r)
        r.mode = "parking"
        if sp is None:
            self.event(r, "parking", "no free spot by an edge reachable now: parks where it stands")
            self.parked(r)
            return
        d, L, tr = sp
        p, h = self.pos(r), self.heading(r)
        h1 = wrap180(h + d)
        mv = dict(kind="park", cell=None, turn=d, L=L, v=5.0, t0=self.t, p0=p, h0=h, h1=h1, end=tr.end,
                  phase="turn", n=r.n_moves + 1, drive_id=None, turn_id=None, t_drive=None)
        if abs(d) >= 3.0:
            mv["turn_id"] = self.send_step(r, "fs %%d T %.2f -" % h1)
        mv["drive_id"] = self.send_step(r, "fs %%d D %.2f %.2f %.2f %.2f %.2f 5.00 0 -" % (
            p[0], p[1], tr.end[0], tr.end[1], h1))
        r.move, r.traj = mv, tr
        self.pump(r)

    # -- (e13) no pictures: a limit per robot from its risk ----------------------------------------------------------
    def on_camera_state(self, state, t=None):
        """The bridge's frame watch: "no_pictures" (no new picture from the camera) or "pictures" (back)."""
        t = self.t if t is None else t
        if state == "no_pictures":
            if self.__dict__.get("nopic_since") is None:
                self.nopic_since = t
        else:
            self.nopic_since = None
            for r in self.R.values():
                if r.__dict__.get("nopic") is not None:
                    self.nopic_release(r, "pictures again")

    def nopic_release(self, r, why):
        st = r.nopic
        r.nopic = None
        if st and st.get("held"):
            r.busy_until = self.t + 0.5
            self.close_break(r)
        if st and st.get("slowed") and r.new_fw and r.mode == "move":
            self.control(r, "fspd 1.00", "fspd")
        self.log("picture_limit", dict(t=round(self.t, 2), robot=r.name, gap_s="", limit_s="", room_cm="",
                                       closing_cms="", action="released: " + why))

    def no_picture_watch(self, r):
        """(e13, replaces e12's fixed 3 s) While the camera sends no pictures: this robot's own limit
        (safety_rules.no_picture_limit: its speed, heading, uncertainty and the room to the edge rule's clearance; a
        robot standing in the middle has none, one driving at an edge almost none). At NOPIC_SLOW_SHARE of it a moving
        robot is slowed to half speed, at the limit it is held (halted) until pictures come back. Every limit used is
        logged (picture_limit.csv) and said once per gap per robot."""
        since = self.__dict__.get("nopic_since")
        if since is None or r.mode in ("out", "gone", "start"):
            return
        last = r.cam_arr if r.cam_arr is not None else (r.cam[0] if r.cam is not None else since)
        gap = self.t - min(last, self.t)          # since its last camera fix
        p, h = self.pos(r), self.heading(r)
        if p is None:
            return
        v = (r.move.get("v", 0.0) * max(0.0, r.scale)) if (r.mode == "move" and r.move is not None and
                                                           r.move.get("phase") != "turn") else 0.0
        lim = SR.no_picture_limit(p[0], p[1], h, v, self.unc(r), self.rect, tracking=self.tracking)
        st = r.__dict__.get("nopic")
        if st is None:
            st = r.nopic = dict(t0=self.t, said=False, slowed=False, held=False, min_limit=lim["limit_s"])
        st["min_limit"] = min(st["min_limit"], lim["limit_s"])
        act = None
        if gap >= lim["limit_s"] and not st["held"] and r.mode == "move":
            act = "held"
            st["held"] = True
            self.halt(r, "no camera pictures for %.1f s, over its limit of %.1f s (%s)" % (gap, lim["limit_s"],
                                                                                         lim["words"]))
            self.end_move(r)
            self.trophy_break(r, "no camera pictures: held until they come back")
        elif gap >= lim["slow_s"] and not st["slowed"] and not st["held"] and r.mode == "move" and r.new_fw:
            act = "slowed"
            st["slowed"] = True
            self.control(r, "fspd 0.50", "fspd")
        if st["held"] or (r.mode != "move" and gap >= 0.0 and lim["limit_s"] != float("inf")):
            r.busy_until = max(r.busy_until, self.t + 0.5)       # no new move until pictures come back
            if r.mode != "move" and not st["held"]:
                st["held"] = True
                act = act or "held"
        if act is not None or not st["said"]:
            st["said"] = True
            self.counts["nopic_" + (act or "watched")] = self.counts.get("nopic_" + (act or "watched"), 0) + 1
            self.log("picture_limit", dict(t=round(self.t, 2), robot=r.name, gap_s=round(gap, 2),
                                           limit_s=lim["limit_s"], room_cm=lim["room_cm"],
                                           closing_cms=lim["closing_cms"], action=act or "watched"))
            self.event(r, "no pictures", "NO CAMERA PICTURES for %.1f s: %s's limit %s (%s)%s" % (
                gap, r.name, "none" if lim["limit_s"] == float("inf") else "%.1f s" % lim["limit_s"], lim["words"],
                {"held": ": HELD until pictures come back", "slowed": ": slowed to half speed"}.get(act, "")))

    # -- (e13) the readable area: a robot that strays out comes back --------------------------------------------
    def strayed(self, r):
        """True when the workspace is limited to the readable area (school) and r's middle is outside it while still
        on table ground (the table's own edge is the edge rules' job, not this)."""
        ws = self.ws
        if not STRAY_RETURN or ws is None or not getattr(ws, "limited", False):
            return False
        p = self.pos(r)
        if p is None:
            return False
        return ws.edge_dist(*p) < 0.0 and ws.edge_dist(*p, raw=True) >= HALF * 0.5

    def stray_return(self, r):
        """(e13) Back into the readable area along ground the robot has just driven: towards its last camera fix well
        inside (else the nearest readable spot). Straight back (reversing) when it faces away from there, else a turn
        on the spot when the table round it is proven (raw field: seen or felt, the readable limit ignored) and then
        forward. Slow (BACK_V); the others' plans and the keep-outs are kept; the edge rule uses the raw table. Returns
        a stop-turn move or None (tried again in a second)."""
        ws = self.ws
        p, h = self.pos(r), self.heading(r)
        if p is None or h is None:
            return None
        tgt = r.__dict__.get("last_in") or ws.nearest_allowed(p[0], p[1], HALF)
        if tgt is None:
            return None
        L = math.dist(p, tgt)
        if L < 1.0:
            tgt = ws.nearest_allowed(p[0], p[1], HALF + 2.0) or tgt
            L = math.dist(p, tgt)
        L = min(L + 2.0, 120.0)                # a little past the target: well inside
        brg = math.degrees(math.atan2(tgt[1] - p[1], tgt[0] - p[0]))
        back = abs(wrap180(brg - (h + 180.0))) <= 30.0
        cands = []
        if back:
            cands.append(("stray return back", 0.0, wrap180(h + 180.0)))
        if ws.edge_dist(*p, raw=True) >= HALF + 1.0:      # its turning circle on table it has proven
            cands.append(("stray return", wrap180(brg - h), brg))
        self.counts["stray_tries"] = self.counts.get("stray_tries", 0) + 1
        for kind, turn, hd in cands:
            for L_ in (L, 0.6 * L):
                tr = Traj.move(self.t + 0.2, p, hd, 0.0, L_, BACK_V, m0=self.margin0(r))
                xs = [p[0] + math.cos(math.radians(hd)) * L_ * k / 10.0 for k in range(11)]
                ys = [p[1] + math.sin(math.radians(hd)) * L_ * k / 10.0 for k in range(11)]
                if min(ws.edge_dist(x_, y_, raw=True) for x_, y_ in zip(xs, ys)) < HALF + 0.5 + (
                        r.bubble if r.lost else 0.0) * 0.5:
                    continue
                if not self.keepout_ok(r, tr):
                    continue
                if any(clearance_profile(tr, o.traj, now=self.t)[-1] < REFLEX_CC for o in self.others(r)):
                    continue
                if kind == "stray return back":
                    tr = Traj.move(self.t + 0.2, p, wrap180(h + 180.0), 0.0, L_, BACK_V, m0=self.margin0(r))
                self.counts["stray_returns"] = self.counts.get("stray_returns", 0) + 1
                self.event(r, "stray return", "outside the area the camera reads at %.0f, %.0f: comes back %.0f cm "
                           "towards %.0f, %.0f (%s), at %.0f cm/s" % (
                               p[0], p[1], L_, tgt[0], tgt[1], "reversing along its track" if turn == 0.0 and
                               kind.endswith("back") else "turning on the spot first", BACK_V))
                self.fallback(r, 0, kind, "back into the readable area")
                return kind, None, turn, L_, BACK_V, tr
        return None

    # -- (e12) one robot at an edge never holds up the others ---------------------------------------------------
    def edge_stuck_why(self, r):
        """Why r stands held at an edge waiting for a person (None: it is not)."""
        if r.mode in ("out", "gone"):
            return None
        if r.__dict__.get("start_held") and not r.__dict__.get("start_ok"):
            return "it was put down at the board's edge and has not been moved in"
        rc = r.__dict__.get("recover")
        if rc is not None and rc.get("hold_until"):
            return "it tilted at the board's edge and found no safe way back"
        if r.__dict__.get("unsure") is not None and r.mode == "idle":
            return "the run cannot tell whether it is clear of the edge and holds it there"
        return None

    def edge_park_watch(self, r):
        """Every half second: a robot held at an edge (edge_stuck_why) for EDGE_PARK_S in a row is parked there (True).
        (e13) With EDGE_PARK off (the default): never parked by a timer; need_hand_watch instead."""
        if not EDGE_PARK:
            return self.need_hand_watch(r)
        why = self.edge_stuck_why(r)
        if why is None:
            r.edge_stuck_since = None
            return False
        if r.__dict__.get("edge_stuck_since") is None:
            r.edge_stuck_since = self.t
            return False
        if self.t - r.edge_stuck_since < EDGE_PARK_S:
            return False
        self.park_at_edge(r, why)
        return True

    def need_hand_watch(self, r):
        """(e13, Nick 10 Nov: robots never give up) A robot held at an edge stays in the run, still and safe (its start
        check, recovery or unsure step keep checking it as before). The moment it is held it says LOUDLY that it needs a
        hand (event NEEDS A HAND, repeated every NEED_HAND_SAY_S while it lasts, in self.need_hand for the state file and
        Mission Control); once moved back in (the checks pass: the camera sees it inside, or its pose clears the edge
        rule) it rejoins by itself and that is said. Its own battery floor still parks it (True), so a home run whose
        other robots are out can end by the battery or the time cap, never held open by it."""
        why = self.edge_stuck_why(r)
        nh = self.__dict__.setdefault("need_hand", {})
        rec = nh.get(r.name)
        if why is None:
            r.edge_stuck_since = None
        elif r.__dict__.get("edge_stuck_since") is None:
            r.edge_stuck_since = self.t
        if why is not None and rec is None and r.__dict__.get("unsure") is not None and \
                not r.__dict__.get("start_held") and self.t - r.edge_stuck_since < 2 * self.UNSURE_RECHECK_S:
            return False       # unsure: it checks again itself first (twice its own re-check time), then asks
        if rec is not None and why is not None and r.move is not None:
            rec["self_moved"] = True           # (e16, 8a) it drove by itself while still held (its own way out)
        if rec is not None and r.cam is not None and self.fresh(r):
            fx = rec.setdefault("fixes", [])
            if not fx or fx[-1][2] != r.cam[0]:
                fx.append((r.cam[1], r.cam[2], r.cam[0]))
                del fx[:-200]
        if why is None:
            if rec is not None:
                if REJOIN_EVIDENCE:
                    # (e16, 8a; home 08:48: "REJOINED ... moved in" 1 s after NEEDS A HAND, nobody there) the reason it
                    # was held went away; that alone is not a hand. Evidence: its own drive out (it freed itself and
                    # stands clear), a lift / put-down, or the camera seeing it move past its own noise, sustained
                    p_ = self.pos(r)
                    clear_ = p_ is not None and edge_dist(self.rect, *p_) >= self.end_need(r)
                    # (e17) a move it made itself since it asked (a creep, never-still) counts as its own way out too
                    own_ = (bool(rec.get("self_moved")) or (self.never_still and r.__dict__.get("creep_n", 0) >
                                                             rec.get("creep_n", 0))) and clear_
                    held_fx = [f for f in rec.get("fixes", []) if f[2] <= rec["t0"] + 2.0] or rec.get("fixes", [])[:5]
                    new_fx = [f for f in rec.get("fixes", []) if f[2] > rec["t0"] + 2.0]
                    import observe as OB
                    ok_, ev_ = OB.moved_in_evidence([f[:2] for f in held_fx], [f[:2] for f in new_fx],
                                                    k_sigma=MOVED_IN_K, lifted=bool(r.__dict__.get("lifted_t", -1e9)
                                                                                     > rec["t0"]))
                    corr = r.__dict__.get("corrected")
                    lifted_ = bool(r.__dict__.get("lifted_t", -1e9) > rec["t0"])
                    if CAMERA_CORRECT and corr is not None and corr["t"] >= rec["t0"] - 1.0 and not lifted_:
                        # (e17, 1d) the camera found it where it stood all along: its pose was corrected, nobody moved
                        # it. Never "moved in" (the home runs' 10 false REJOINED); it carries on from the camera's spot
                        nh.pop(r.name, None)
                        self.counts["corrected_rejoins"] = self.counts.get("corrected_rejoins", 0) + 1
                        r.rejoined_at = self.pos(r)
                        self.event(r, "CORRECTED", "nobody moved %s: the camera found it at %.0f, %.0f (its own pose "
                                   "was %.0f cm off); its pose corrected, it carries on (held %.0f s%s)" % (
                                       r.name, corr["x"], corr["y"], corr["d_cm"], self.t - rec["t0"],
                                       ", creeping meanwhile" if rec.get("self_moved") else ""))
                        return False
                    if own_:
                        nh.pop(r.name, None)
                        self.counts["freed_itself"] = self.counts.get("freed_itself", 0) + 1
                        self.event(r, "FREED ITSELF", "%s drove itself clear of the edge rule (%.0f s after asking for a "
                                   "hand): the run goes on" % (r.name, self.t - rec["t0"]))
                        return False
                    if not (ok_ and clear_):
                        if rec.get("why_wait") != ev_:
                            rec["why_wait"] = ev_
                            self.counts["rejoin_refused"] = self.counts.get("rejoin_refused", 0) + 1
                            self.log("rejoin_check", dict(t=round(self.t, 2), robot=r.name, moved=int(ok_),
                                                          clear=int(clear_), why=ev_))
                        return False           # still held: no evidence a person moved it (8b keeps it pinned)
                    rec["evidence"] = ev_
                nh.pop(r.name, None)
                self.counts["hand_rejoined"] = self.counts.get("hand_rejoined", 0) + 1
                r.rejoined_at = self.pos(r)    # (8b) pinned until it has driven away from there
                self.event(r, "REJOINED", "%s was moved in and rejoins the run by itself (it waited %.0f s for a hand%s)"
                           % (r.name, self.t - rec["t0"], ("; " + rec["evidence"]) if rec.get("evidence") else ""))
            return False
        p = self.pos(r)
        if rec is None:
            d, side = self.edge_side(p) if p is not None else (None, "an edge")
            words = ("%s NEEDS A HAND%s: %s. It stays still and safe and keeps checking; move it a hand's width (10 cm) "
                     "onto the board and it rejoins by itself. The others carry on." % (
                         r.name, (" at %.0f, %.0f (%.0f cm from %s)" % (p[0], p[1], d, side)) if p is not None else "",
                         why))
            rec = dict(creep_n=r.__dict__.get("creep_n", 0), robot=r.name, x=round(p[0], 1) if p else None,
                       y=round(p[1], 1) if p else None,
                       edge_cm=round(d, 1) if d is not None else None, side=side, why=why, t0=round(self.t, 1),
                       said=self.t, words=words, n_moves=r.n_moves)
            nh[r.name] = rec
            self.counts["need_hand"] = self.counts.get("need_hand", 0) + 1
            self.event(r, "NEEDS A HAND", words)
        elif self.t - rec["said"] >= NEED_HAND_SAY_S:
            rec["said"] = self.t
            self.event(r, "NEEDS A HAND", "still waiting (%.0f s): %s" % (self.t - rec["t0"], rec["words"]))
        if r.battery is not None and r.battery <= self.floor:
            nh.pop(r.name, None)
            r.out_why = "battery %d %% while waiting for a hand at the edge" % r.battery
            self.close_question(r, "its battery reached the floor")
            self.parked(r)                 # sleeps (park_sleep) where it stands: its own battery rule
            return True
        return False

    def needs_hand(self):
        """(e13) [record] of robots now waiting for a hand at an edge."""
        return list((self.__dict__.get("need_hand") or {}).values())

    def edge_side(self, p):
        """(distance to the nearest edge of the typed board, words for that edge)."""
        b = getattr(self.rect, "board", None) or self.rect
        x0, y0, x1, y1 = (float(v) for v in tuple(b)[:4])
        sides = [(p[0] - x0, "the x = %.0f edge" % x0), (x1 - p[0], "the x = %.0f edge" % x1),
                 (p[1] - y0, "the y = %.0f edge" % y0), (y1 - p[1], "the y = %.0f edge" % y1)]
        return min(sides)

    def park_at_edge(self, r, why):
        p = self.pos(r)
        r.mode, r.out_why = "out", "held at an edge (%s)" % why
        if p is not None:
            r.traj = Traj.stand(self.t, *p, m=self.margin0(r) + 1.0)
        r.move = None
        self.out(r, "stop")
        self.close_question(r, "parked at the edge for the rest of the run")
        d, side = self.edge_side(p) if p is not None else (None, "an edge")
        dw = edge_dist(self.rect, *p) if p is not None else None
        ws_words = ("; %.0f cm from the edge of the ground proven safe" % dw) if (
            dw is not None and d is not None and abs(dw - d) > 3.0) else ""
        words = ("%s IS PARKED AT THE BOARD'S EDGE%s: %s for %.0f s. It stays still for the rest of this run and the "
                 "others carry on. SOMEONE MUST MOVE IT a hand's width onto the board before the next run." % (
                     r.name, (" at %.0f, %.0f (%.0f cm from %s%s)" % (p[0], p[1], d, side, ws_words))
                     if p is not None else "", why, self.t - (r.__dict__.get("edge_stuck_since") or self.t)))
        rec = dict(robot=r.name, x=round(p[0], 1) if p else None, y=round(p[1], 1) if p else None,
                   edge_cm=round(d, 1) if d is not None else None, side=side, why=why, t=round(self.t, 1), words=words)
        self.__dict__.setdefault("edge_parked", []).append(rec)
        self.counts["edge_parked"] = self.counts.get("edge_parked", 0) + 1
        self.event(r, "PARKED AT AN EDGE", words)

    def parked(self, r):
        p = self.pos(r)
        r.mode = "out"
        r.out_why = r.out_why or "battery"
        if p is not None:
            r.traj = Traj.stand(self.t, *p, m=self.margin0(r) + 1.0)
        self.out(r, "stop")
        self.arc_save(r)
        if self.park_sleep:
            self.out(r, "sleep 0")
        self.event(r, "out", "parked at %s and stopped (%s)" % ("%.1f, %.1f" % p if p else "?", r.out_why))

    def arc_save(self, r):
        """(11-05d) A parked curve robot: the curves' calibration from this run saved on it (set + save, as the
        calibration's values), when there are ARC_FIT_MIN fitted pieces and both medians are within ARC_FIT_CI.
        Switch: arc_save (freeroam.py --no-arc-save)."""
        res = self.arc_result(r) if r.curve else None
        if res is None:
            return
        kr, kh, dr, dh, n = res
        cur = r.arc_scales or (1.0, 1.0)
        ok, ks, ds = arc_agree(res, cur)
        words = ("curves' calibration from %d pieces: curvature x%.3f (+-%.3f), length x%.3f (+-%.3f) of what was asked"
                 "; arc_k_scale %.4f -> %.4f, arc_d_scale %.4f -> %.4f%s" % (
                     n, kr, kh, dr, dh, cur[0], ks, cur[1], ds, "" if r.arc_scales else " (its values in use were "
                     "not heard: taken as 1)"))
        self.log("arcfit", dict(t=round(self.t, 2), robot=r.name, k="median", L="", v="", R_fit="", k_ratio=round(kr, 4),
                                length_ratio=round(dr, 4), n=n))
        if ok and getattr(self, "arc_save_on", True):
            self.out(r, "set arc_k_scale %.4f" % ks)
            self.out(r, "set arc_d_scale %.4f" % ds)
            self.out(r, "save")
            self.event(r, "arc calibration", words + ": saved on the robot")
        else:
            self.event(r, "arc calibration", words + (": NOT saved (%s)" % (
                "switched off" if ok else "too few pieces or too spread")))

    # -- every tick ----------------------------------------------------------------------------------------------
    def tick(self, t):
        self.t = t
        if self.t_start is None:
            self.t_start = t
        for r in self.R.values():
            self.watch(r)
        self.camera_timing()
        if self.keepouts and int(t * 2) != int((t - 0.05) * 2):
            self.clear_keepouts()
        self.reflex()
        self.adj_watch()
        half = int(t * 2) != int((t - 0.05) * 2)
        for r in self.R.values():
            if r.__dict__.get("fpose_pending") is not None:
                self.fpose_pending_watch(r)            # (e17, 1e) a re-seed waiting for the turn to end
            self.late_fix_watch(r)                     # (e17, 2) late fixes: slow or wait
            self.gap_forward(r)                        # (e09, B2) the filter's position to the robot in camera gaps
            self.gyro_watch(r)                         # (e09, B3) switch, off by default
            if getattr(r, "want_wiggle", None) is not None and r.mode == "idle" and self.pos(r) is not None:
                r.want_wiggle = None
                r.wiggle_due = 0.0
                self.find_me(r)
            if getattr(r, "held", None):
                self.held_check(r)                     # (e08, A7) held as an obstacle, checked, rejoins
                if r.mode not in ("gone",):
                    self.pump(r)
                continue
            if r.__dict__.get("probe_active") is not None:
                self.probe_over(r)                     # (e10, C2) every tick: the return before any other move
            if half:
                self.edge_watch(r)                     # (e08, A2) sure / unsure near an edge
                self.felt_mark(r)                      # (e10, C2) the proven workspace grows under its wheels
                if NOPIC_RULE and self.__dict__.get("nopic_since") is not None:
                    self.no_picture_watch(r)           # (e13) no pictures: this robot's own risk-based limit
                if CONTACT_WATCH:
                    self.contact_watch(r)              # (e16, 8c) pushed while standing, or blocked while driving
                if self.edge_park_watch(r):            # (e12) held at an edge too long: parked, the others go on
                    continue
            if r.mode == "idle" and t >= r.busy_until and self.pos(r) is not None and not r.silent and \
                    not self.start_check(r):
                pass                                   # (e10, run 2 incident d) put down over or at an edge: held
                                                       # (before anything else, the unsure-near-an-edge step too)
            elif r.mode == "idle" and r.__dict__.get("hd_drive") is not None and not r.silent:
                self.ask_heading_click(r)              # (e13) its heading from a second click: it waits for it
                r.busy_until = t + 0.5
            elif r.mode == "idle" and t >= r.busy_until and r.__dict__.get("hd_confirm") is not None and \
                    not r.silent and r.lost is None and self.pos(r) is not None and not getattr(r, "tilted", False) \
                    and r.__dict__.get("unsure") is None:
                self.heading_confirm_step(r)           # (e17, 2) a camera-confirmed heading before the next move
            elif r.mode == "idle" and t >= r.busy_until and not r.silent and self.pos(r) is not None and \
                    not getattr(r, "tilted", False) and self.strayed(r):
                res_s = self.stray_return(r)           # (e13) outside the readable area: back along ground it drove
                if res_s is not None:
                    r.coast = None
                    self.stopturn_move(r, res_s)
                else:
                    r.busy_until = t + 1.0
            elif r.mode == "idle" and t >= r.busy_until and r.__dict__.get("probe_ret") and not r.silent and \
                    self.pos(r) is not None and not getattr(r, "tilted", False):
                res_r = self.probe_return(r)           # (e10, C2) back from a probe first, lost or not (the camera
                if res_r is not None:                  # may not see there: it retraces its own path)
                    r.coast = None
                    self.stopturn_move(r, res_r)
                else:
                    r.busy_until = t + 0.5
            elif r.mode == "idle" and t >= r.busy_until and r.__dict__.get("unsure") and not r.silent and \
                    self.pos(r) is not None and not getattr(r, "tilted", False):
                self.unsure_step(r)
            elif r.mode == "idle" and t >= r.busy_until and self.pos(r) is not None and not r.silent and r.curve and \
                    not r.lost_hold and not (r.battery is not None and r.battery <= self.floor) and \
                    not (r.lost is not None and (r.bubble > 2 * BUBBLE_CAP if BLIND_RULE == "cap" else
                                                 self.blind_stop(r, self.pos(r), 5.0))):
                self.start_curve(r)                    # (11-05d) never waits: from the coast or the stand
            elif r.mode == "idle" and t >= r.busy_until and self.pos(r) is not None and not r.silent:
                if r.lost is not None and (r.bubble > BUBBLE_CAP if BLIND_RULE == "cap" else
                                           self.blind_stop(r, self.pos(r), 5.0)):
                    if BLIND_RULE != "cap":
                        self.ask_place(r)              # (e13, Nick 12:56) a person clicks it on a picture
                    if self.never_still and t - r.__dict__.get("creep_t", -1e9) >= self.still_pause(r):
                        r.creep_t = t                  # (e17) creeps toward being found, never stands
                        if not self.creep_step(r, "lost at its blind limit"):
                            self.find_me(r)
                    else:
                        self.find_me(r)
                elif r.battery is not None and r.battery <= self.floor:
                    r.out_why = "battery %d %%" % r.battery
                    self.go_park(r, "battery %d %% (floor %d %%)" % (r.battery, self.floor))
                elif not r.lost_hold:
                    self.start_move(r)
            self.never_still_watch(r)                  # (e17, Nick 12 Nov) held robots creep, never settle at 0
            if r.curve and r.mode == "move" and r.move is not None and r.move.get("curve") and not r.silent:
                self.queue_watch(r)                    # (e10, run 2 incident a, c) every tick: still on plan?
            if r.curve and r.mode == "move" and r.move is not None and r.move.get("curve") and not r.queue and \
                    not r.silent:
                self.plan_next(r)                      # (11-05d) the next piece queued while this one runs
            if r.mode not in ("gone",):
                self.pump(r)
            if r.mode not in ("out", "gone", "start") and t - r.last_sent > HEARTBEAT_S:
                self.out(r, "status")
                r.last_status = t
            elif r.mode not in ("gone", "start") and t - r.last_status > STATUS_S:
                self.out(r, "status")
                r.last_status = t
        if t - self.last_motion >= 0.5:
            self.last_motion = t
            self.motion_row()

    def watch(self, r):
        if r.mode in ("out", "gone", "start") or getattr(r, "held", None):
            return
        t = self.t
        # silence
        if r.heard is not None and t - r.heard > SILENT_S and not r.silent:
            r.silent, r.silent_t = True, t
            self.event(r, "silent", "nothing heard for %.0f s: kept as an obstacle where the camera sees it" % SILENT_S)
            p = self.pos(r)
            if p:
                r.traj = Traj.stand(t, *p, m=self.margin0(r))
        if r.silent:
            if r.cam is not None and self.fresh(r):
                r.traj = Traj.stand(t, r.cam[1], r.cam[2], m=self.margin0(r))
            if t - r.silent_t > OUT_SILENT_S:
                if self.silent_out:
                    r.mode, r.out_why = "out", "silent"
                    self.event(r, "out", "silent for %.0f s: out of the run (an obstacle where it is)" % OUT_SILENT_S)
                else:
                    # (e08, A7, Nick 30 Oct "no robot permanently out") held, checked, rejoins
                    self.hold_robot(r, "silent", "silent for %.0f s" % OUT_SILENT_S)
            return
        # nothing from the robot for a moment while it should be moving: it may have stopped where the camera has it
        if r.own is not None and t - r.own[0] > 1.0 and r.mode == "move" and r.cam is not None and \
                not getattr(r, "stale_cover", False):
            r.stale_cover = True
            r.traj = concat(Traj.stand(t, r.cam[1], r.cam[2], m=self.margin0(r)), r.traj) if r.traj is not None \
                else Traj.stand(t, r.cam[1], r.cam[2], m=self.margin0(r))
        elif r.own is not None and t - r.own[0] <= 1.0:
            r.stale_cover = False
        # lost by the camera
        if r.lost is None and r.cam is not None and not self.fresh(r):
            self.start_lost(r)
        if r.mode == "move" and r.move is not None and r.move.get("kind") not in ("stray return", "stray return back",
                                                                               "probe", "probe return") \
                and STRAY_RETURN and self.strayed(r):
            # (e13, e12's school practice: robots roamed past x 254 cm, where the camera cannot read, and stood lost)
            self.ease_stop(r, "outside the area the camera reads") if r.lost is not None else \
                self.halt(r, "outside the area the camera reads: comes back first")
            if r.lost is None:
                self.end_move(r)
            return
        if r.lost is not None and r.mode == "move" and r.move is not None:
            p = self.pos(r)
            if r.move.get("curve"):
                # (11-05d) a curve robot's plans carry the bubble in their edge margins and it loops in its own disc
                # when the bubble passes BUBBLE_CAP (find-me loop): it eases to a stop only when far more lost
                if p and (r.bubble > 2 * BUBBLE_CAP if BLIND_RULE == "cap" else
                          self.blind_stop(r, p, r.move.get("v", 5.0) * max(r.scale, 0.0))):
                    self.ease_stop(r, "bubble %.1f cm" % r.bubble)
            elif p and ((r.bubble > BUBBLE_CAP or edge_dist(self.rect, *p) - r.bubble < TURN_EDGE_CC)
                        if BLIND_RULE == "cap" else self.blind_stop(r, p, r.move.get("v", 5.0) * max(r.scale, 0.0))) \
                    and r.move["kind"] not in ("wiggle", "probe", "probe return"):
                # (e10) a probe goes out of the camera's view on purpose, and its return retraces the strip it proved
                self.ease_stop(r, "bubble %.1f cm" % r.bubble)
        # stuck: driving, wheels on, no camera travel
        mv = r.move
        if mv is not None and mv["phase"] == "drive" and r.scale > 0.3 and r.lost is None and mv.get("t_drive") and \
                t - mv["t_drive"] > STUCK_S + 1.0:
            hist = [q for q in r.cam_hist if q[0] > t - STUCK_S]
            if len(hist) >= 5 and math.dist(hist[0][1:], hist[-1][1:]) < 1.0 and r.own and r.own[4] in ("drive",
                                                                                                         "curve"):
                self.event(r, "stuck", "drive commanded, the camera saw it move %.1f cm in %.0f s: new move" % (
                    math.dist(hist[0][1:], hist[-1][1:]), STUCK_S))
                self.halt(r, "stuck")
                self.end_move(r)
        # (11-05d) a curve piece well past its time (off its path, slowed, bent): re-aimed from where it is with a new
        # piece (farc), no stop
        if REAIM and mv is not None and mv.get("curve") and mv.get("t_drive") is not None and r.scale > 0 and \
                t - mv["t_drive"] > 1.5 * mv["L"] / max(0.5, mv["v"] * r.scale) + 3.0 and r.lost is None:
            p, h = self.pos(r), self.heading(r)
            res = self.pick_curve(r, p[0], p[1], h, t)
            if res is not None:
                kind, cell, k, L, v, side, _ = res
                self.event(r, "overdue piece", "piece %d ran %.0f s (planned %.0f): re-aimed from where it is "
                           "(farc, no stop)" % (mv["drive_id"], t - mv["t_drive"], mv["L"] / max(0.5, mv["v"])))
                self.control(r, "farc %.5f %.1f" % (k, L), "farc")
                for q in r.queue:
                    st = r.steps.get(q["drive_id"])
                    if st:
                        st["status"] = "dropped"
                r.queue = []
                ex, ey, eh = arc_at(p[0], p[1], h, k, L)
                mv.update(k=k, L=L, x=p[0], y=p[1], h=h, end=(float(ex), float(ey)), h1=float(eh), side=side,
                          t_drive=t, t0=t, kind=mv["kind"] + "+reaimed")
                if cell is not None:
                    self.cmenu.use(cell, r.name)
                self.counts["reaimed"] = self.counts.get("reaimed", 0) + 1
                self.retraj_curve(r)
        # a move that takes far too long
        if mv is not None:
            est = turn_s(mv["turn"]) + (drive_s(mv["L"], mv["v"]) if mv["L"] else 0.0)
            if t - mv["t0"] > 3.0 * est + 15.0 and not mv.get("paused"):
                self.event(r, "slow move", "move %d took %.0f s (estimate %.0f): ended (%s, scale %.2f, own %s)" % (
                    mv["n"], t - mv["t0"], est, mv["kind"], r.scale, r.own[4] if r.own else "-"))
                self.halt(r, "move timed out")
                self.end_move(r)

    # -- (e13, Nick 12:56) a person clicks the held robot on a camera's picture ----------------------------------------
    def ask_place(self, r):
        """Held because its error sum could reach an edge: ask for a click (once per hold; place_fn set by the host:
        the request file for Mission Control and the terminal's line). Said loudly either way."""
        if r.__dict__.get("place_asked"):
            return
        r.place_asked = self.t
        p = self.pos(r)
        self.counts["place_asked"] = self.counts.get("place_asked", 0) + 1
        self.event(r, "NEEDS A HAND", "%s holds: its possible error could reach an edge. Click it on any camera's "
                   "picture in Mission Control (or put it on a sticker and click that point on the map): it moves again "
                   "from there" % r.name)
        fn = getattr(self, "place_fn", None)
        if fn is not None:
            try:
                fn(r.name, p)
            except Exception:  # noqa: BLE001
                pass

    def heading_drive_cm(self, r, p, sig, v=5.0):
        """(e13, Nick 13:01) How far it may drive straight to show its heading: as far as the fresh error sum (sig, grown
        0.07 cm per cm driven) + half the body + its stop stay inside the edge rule in ANY direction (its heading is
        the unknown): L = (edge distance - sig - half diagonal - stop) / (1 + 0.07). The longer, the better the heading
        (+- atan(sig sqrt 2 / L))."""
        e = edge_dist(self.rect, *p)
        L = (e - sig - HALF - SR.stop_cm(v)) / (1.0 + BLIND_PER_CM)
        for o in self.others(r):             # never into another robot's spot: the circles kept apart
            q = self.pos(o)
            if q is not None:
                L = min(L, math.dist(p, q) - 2 * HALF - PLAN_GAP - sig)
        return max(0.0, L)

    def ask_heading_click(self, r):
        hd = r.__dict__.get("hd_drive")
        if hd is None or hd.get("asked"):
            return
        hd["asked"] = self.t
        r.place_asked = self.t
        self.event(r, "NEEDS A HAND", "%s drove %.0f cm straight to show its heading: click it again on a picture "
                   "(or confirm the motion detector's spot): the line between the clicks is its heading" % (
                       r.name, hd["L"]))
        fn = getattr(self, "place_fn", None)
        if fn is not None:
            try:
                fn(r.name, hd["p0"], stage="heading", first=hd["p0"], drive_cm=hd["L"], sigma_cm=hd["sig"])
            except TypeError:
                fn(r.name, hd["p0"])
            except Exception:  # noqa: BLE001
                pass

    def on_place(self, name, ans):
        """A person's click (place_click answer: x, y, sigma_cm, heading or None): the robot's position is set there and
        its error sum restarts at the click's own accuracy; the heading too if the nose was clicked (else its gyro's
        heading, whose doubt keeps growing slowly). The next camera fix's residual is logged by the host."""
        r = self.R.get(name)
        if r is None:
            return
        x, y, sig = float(ans["x"]), float(ans["y"]), float(ans["sigma_cm"])
        h = ans.get("heading")
        h = float(h) if h is not None else self.heading(r)
        if ans.get("stage") == "heading":
            r.hd_drive = None
            if ans.get("heading") is not None:
                self.event(r, "heading from two clicks", "%.0f deg +- %.1f (clicks %.1f cm apart)" % (
                    h, ans.get("heading_pm") or 0.0, ans.get("drive_seen_cm") or 0.0))
                self.counts["heading_clicks"] = self.counts.get("heading_clicks", 0) + 1
        own0 = (r.own[1], r.own[2]) if r.own is not None else None
        r.place = dict(t=self.t, x=x, y=y, own=own0, how=ans.get("how", "click"))
        self.out(r, "fpose %.2f %.2f %.2f %.2f %.2f" % (x, y, h, max(1.0, sig), 3.0 if ans.get("heading") is not None
                                                        else 10.0))
        if r.own is not None:
            r.own = (r.own[0], x, y, h) + tuple(r.own[4:])
        if r.lost is not None:
            r.lost.update(base=sig, drive=0.0, turn=0.0, after_turn=0.0, at=(x, y))
        r.bubble = sig
        r.pc = None
        try:
            if r.fp is not None:
                self.fp_fix(r, self.t, x, y, ans.get("heading"), sig, "click")
        except Exception:  # noqa: BLE001
            pass
        r.place_asked = None
        r.busy_until = self.t
        if r.stopped_hard:
            self.close_break(r)
        if ans.get("stage", "position") == "position" and HEADING_CLICKS:
            # (Nick 13:01) its heading: a straight drive as long as is safe, then a second click
            L = self.heading_drive_cm(r, (x, y), sig)
            if L >= 2.0 * sig:               # shorter, the two clicks could not tell a heading at all (+- > 35 deg)
                tr = Traj.move(self.t + 0.2, (x, y), h, 0.0, L, BACK_V, m0=self.margin0(r))
                if all(clearance_profile(tr, o.traj, now=self.t)[-1] >= REFLEX_CC for o in self.others(r)):
                    r.hd_drive = dict(p0=(x, y), L=L, sig=sig, t=self.t)
                    mv = dict(kind="heading drive", cell=None, turn=0.0, L=L, v=BACK_V, t0=self.t, p0=(x, y), h0=h,
                              h1=h, end=tr.end, phase="turn", n=r.n_moves + 1, drive_id=None, turn_id=None,
                              t_drive=None, adj_scale=1.0, rev=False)
                    mv["drive_id"] = self.send_step(r, "fs %%d D %.2f %.2f %.2f %.2f %.2f %.2f 0 -" % (
                        x, y, tr.end[0], tr.end[1], h, BACK_V))
                    r.move, r.traj, r.mode = mv, tr, "move"
                    r.n_moves += 1
                    self.event(r, "heading drive", "%.1f cm straight (the most its fresh +- %.1f cm and body allow): "
                               "its heading will be +- %.0f deg from the two clicks" % (
                                   L, sig, math.degrees(math.atan(sig * math.sqrt(2) / L))))
                    self.pump(r)
        self.counts["placed"] = self.counts.get("placed", 0) + 1
        self.event(r, "placed by a click", "%s set at %.1f, %.1f (+- %.1f cm%s, %s): it moves again with a new running "
                   "total" % (r.name, x, y, sig, ", heading %.0f deg" % h if ans.get("heading") is not None else "",
                              ans.get("how", "click")))

    def blind_stop(self, r, p, v=5.0):
        """(e13, CLAUDE.md "never give up") True when this lost robot's whole possible error could reach an edge: its
        bubble (grown with every cm and degree blind) + half its body + its stop at speed v, against its distance to the
        nearest edge (or unproven ground). Until then it keeps moving on its own wheels and gyro."""
        if p is None or r.lost is None:
            return False
        self.counts["blind_checks"] = self.counts.get("blind_checks", 0) + 1
        return edge_dist(self.rect, *p) - r.bubble < HALF + SR.stop_cm(max(0.0, v) if v and v > 0.3 else None)

    def ease_stop(self, r, why):
        self.why_zero(r, "lost: eases to a stop (%s)" % why)
        if r.new_fw:
            self.control(r, "fspd 0.30", "fspd")
        self.halt(r, "eases to a stop: " + why)
        self.event(r, "find me", "bubble too big (%s): eases to a stop, then wiggles" % why)
        self.end_move(r)

    def find_me(self, r, safe_checked=False):
        if self.t < r.wiggle_due:
            return
        if not safe_checked and not self.can_turn_here(r):
            r.wiggle_due = self.t + 3.0
            if not r.stopped_hard:
                self.trophy_break(r, "lost near an edge: stands until seen")
            return
        p, h = self.pos(r), self.heading(r)
        mv = dict(kind="wiggle", cell=None, turn=WIGGLE_DEG, L=0.0, v=5.0, t0=self.t, p0=p, h0=h, h1=h, end=p,
                  phase="turn", n=r.n_moves + 1, drive_id=None, turn_id=None, t_drive=None)
        self.send_step(r, "fs %%d T %.2f -" % wrap180(h + WIGGLE_DEG))
        self.send_step(r, "fs %%d T %.2f -" % wrap180(h - WIGGLE_DEG))
        mv["turn_id"] = self.send_step(r, "fs %%d T %.2f -" % wrap180(h))
        r.move, r.mode = mv, "move"
        r.traj = Traj.stand(self.t, *p, m=self.margin0(r))
        r.n_moves += 1
        r.wiggle_due = self.t + 4.0
        self.fallback(r, 6, "find-me wiggle", "lost, bubble %.1f cm over %.0f" % (r.bubble, BUBBLE_CAP))
        self.why_zero(r, "find-me wiggle (lost)")
        self.pump(r)

    def motion_row(self):
        for r in self.R.values():
            h = [q for q in r.own_hist if q[0] >= self.t - 0.6]
            trans = anym = 0
            if r.own is not None and self.t - r.own[0] < 1.0 and len(h) >= 2 and r.mode not in ("out", "gone"):
                d = math.dist(h[0][1:3], h[-1][1:3])
                dth = abs(wrap180(h[-1][3] - h[0][3]))
                dt = max(0.1, h[-1][0] - h[0][0])
                trans = int(d / dt >= 1.0)
                anym = int(trans or dth / dt >= 4.0)
            self.zero_watch(r, trans)
            p = self.pos(r)
            self.log("motion", dict(t=round(self.t, 2), robot=r.name, any=anym, translating=trans,
                                    x=round(p[0], 1) if p else "", y=round(p[1], 1) if p else "",
                                    bubble=round(r.bubble, 2) if r.lost else 0.0, mode=r.mode,
                                    lost=int(r.lost is not None), battery=r.battery if r.battery is not None else "",
                                    style="curve" if r.curve else "stopturn"))

    def zero_watch(self, r, trans):
        """(11-05d) Speed-zero moments from the robot's own wheels (FOLLOW, 10 a second): its speed under 1 cm/s
        (translating 0) while in the roam; each one logged (zeros.csv) with why: the latest reason the controller
        noted in the 3 s before (a stop-turn move, a reverse, a hard stop, the watchdog, a coast that ran out ...)."""
        if r.mode in ("out", "gone", "start") or r.own is None or self.t - r.own[0] > 1.0:
            if r.zero is not None:
                self.close_zero(r)
            return
        if not trans and r.zero is None:
            zw = r.zero_why
            why = zw[0] if zw and self.t - zw[1] <= 3.0 else ("stop-turn (turns on the spot)" if not r.curve else
                                                             "unexplained")
            r.zero = dict(t=self.t, why=why)
        elif trans and r.zero is not None:
            self.close_zero(r)

    def close_zero(self, r):
        z = r.zero
        r.zero = None
        dur = self.t - z["t"]
        r.zeros.append((z["t"], dur, z["why"]))
        key = z["why"].split(":")[0].split(" (")[0]
        self.counts["zeros"][key] = self.counts["zeros"].get(key, 0) + 1
        self.log("zeros", dict(t=round(z["t"], 2), robot=r.name, style="curve" if r.curve else "stopturn",
                               dur_s=round(dur, 2), why=z["why"]))

    def begin(self, name, held=None):
        """The robot is following (run follow) and seen: it joins the roam. held: (e08, A7) "heading" when its heading
        crawl failed: it joins held as an obstacle, and a nudge the camera watches finds its heading."""
        r = self.R[name]
        r.mode = "idle"
        if held:
            self.hold_robot(r, held, "its heading is not known (the heading crawl failed)" if held == "heading" else
                            str(held))
        r.curve = self.uses_curves(r)
        r.zero_why = ("the start (standing)", self.t + 1.0)
        self.event(r, "motion", "curves (continuous: firmware %s)" % r.fw if r.curve else
                   "stop-turn-drive (%s)" % ("chosen" if self.motion_style == "stopturn" else
                                            "firmware %s is older than %s" % (r.fw or "?", ARC_FW)))
        p = self.pos(r)
        if p:
            r.traj = Traj.stand(self.t, *p, m=self.margin0(r))

    def sent(self, name):
        """The runner forwarded something to this robot outside the controller (a camera fix): heartbeat met."""
        r = self.R.get(name)
        if r is not None:
            r.last_sent = self.t

    def all_out(self):
        return all(r.mode in ("out", "gone") for r in self.R.values())
