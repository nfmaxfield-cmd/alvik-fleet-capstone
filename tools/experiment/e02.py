#!/usr/bin/env python3
"""E02 accuracy experiment: the laptop runner. Double-click run-e02.bat, or:

    python tools/experiment/e02.py --pi st1maxfini.local            # a new session (about 2 hours)
    python tools/experiment/e02.py --resume                          # carry on the last session after a crash
    python tools/experiment/e02.py --dry-run --blocks 1 --routes 1,2 # the whole thing against the simulator

Plan: docs/build-plans/accuracy-experiment-E02.md. What it does, in order:
  1. Setup checks, each in plain words: the Pi recorder (both cameras answering, notes heard), the floor
     calibration (the studio's cameras/<camera>.json, or one made here from the board's corners: it
     offers), the two blocked boards found in the picture and their floor lines measured, every route fits,
     the robots connected (battery), each robot found by the camera (a short drive), then parked.
  2. A find check: one `run find` per robot, so a firmware problem shows before 2 hours start, and the
     camera measures where each robot really touched each board (a second measure of the board lines).
  3. The 2-minute naming check: every robot stands still blinking its code (`run gyro` for 120 s, which
     also measures each gyro's drift) while tools/live's blink tracker names them. Recorded only.
  4. The schedule: balanced, shuffled blocks of U / C / K / KC x the three routes x the robots, block 0
     all U. One robot moves at a time. Per run: corrections for the condition, `run find`, `run e02`
     (K: the laptop steers it leg by leg; KC: camera fixes 3 a second), the camera measures start, far
     point and end, marks go into both camera videos, then it is parked again by camera. After each U run
     the corrections are re-learned (gyro scale, drift, wheels) and the latest are used for C and KC.
  5. Every robot is parked again by camera after its run. A run that NEEDED RECOVERY (the camera lost it, it
     left the board, a safety stop cut its route short, or it ended more than 40 cm from its start) is noted
     as such (runs.csv `recovery`), never as an outlier: outliers are the analysis's statistical call within a
     condition. A camera watchdog stops any robot heading for an open edge or a parked robot. Nick is called
     only when a robot is stuck or off the board, with exact steps; with nobody answering for --help-wait-min
     that robot is put aside and the others carry on.
Everything is saved after every step (e02_state.json), so after a crash `--resume` carries on: an
interrupted run is redone.

Files in the session folder (logs/e02-<date-time>/): e02_state.json, runs.csv (one row per run: camera
start / far / end, errors, flags, corrections used), track.csv (the moving robot's camera positions),
fits.csv (every self-calibration), e02_log.txt (everything the console said), naming_check.json,
setup.json, robot logs in robot-logs-<n>/ (the console's CSVs: raw, bump, find, e02, leg ...).

PROTOCOL: firmware 2026-10-22a (docs/data-formats.md "E02", firmware/home_lab/e02.h); the runner refuses other
firmware. What this runner sends:
  set safe_edge 3, edge_dir 0, edge_thr 150, edge_n 3, edge_bot_cm 11, bumper_cm 0, e02_face_cm 0  (SAFETY: at setup, checked by
      `get`, and with every find / route, checked in its PARAM records; the firmware refuses to drive without
      the edge guard)
  --switches-back: also set sw_on 1, sw_back 1, e02_find_reverse 1, e02_back_face_cm 6.10 (BACK_SETTINGS: read back
      at setup, sent again before every find): the find backs into the boards (firmware 2026-10-25b; FIND reverse)
  calibrate (once after power-on; calib_spin_deg 0, calib_roll_cm 0: only the compass wiggle)
  set gyro_scale / gyro_drift_dps / wheel_scale  (finds, transits and moves: the latest learned, else the priors;
      K: 1, 0, 1) and e02_route_gyro_scale / _gyro_drift_dps / _wheel_scale (U: 1, 0, 1; C, KC: learned): the
      firmware switches to these at the route's start, so only the route runs on the condition's corrections. `set`, not `corr reset`: same values, no rewrite of the saved corrections
  set e02_xboard_side / e02_yboard_side (-1 | +1), e02_xboard_cm / e02_yboard_cm (camera-measured faces),
      e02_start_x / e02_start_y (route start spot), e02_route_deg (straight: along the diagonal; L, square:
      away from the y-board), e02_l_turn (towards the middle), e02_leg_pause_ms 700 (all mode-0 runs: KC
      fixes are only used while still), e02_transit_max_cm 140,
      e02_pause_ms 1000, e02_route, e02_mode
  fix x y th|- t_wall   before every find, every set of idle legs and every backwards leg (replaces the pose; it
      also resets the firmware's blind-backing allowance); KC: up to 3 a second while it stands still, from
      pictures taken after it stopped, heading only when the camera just saw a straight stretch and RAW yaw
      says no turn since
  run find; run e02; go d id / turn deg id (idle legs, and mode 1 legs in K); mark start|far|end id (K)
  Records read by column name: RUNINFO, RUNEND, PARAM, RAW (ms, x_cm, y_cm, yaw_deg, th_deg), FIND (result,
  after_th), BUMP (board, ms_contact), E02 (start_x/y, far_goal_x/y, end_x/y, ms_start, ms_far,
  fixes_used), LEG (id, run, result or state), GYRO (raw_drift_dps).
"""
import argparse
import csv
import datetime as dt
import glob
import json
import math
import os
import subprocess
import re
import signal
import sys
import threading
import time
from collections import defaultdict, deque

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, TOOLS):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np  # noqa: E402

import e02_plan as PL  # noqa: E402
import e02_steer as ST  # noqa: E402
import e02_fit as FIT  # noqa: E402
import e02_camera as CAM  # noqa: E402
import e02_map as MAP  # noqa: E402
import e02_explore as EX  # noqa: E402
import e02_edgefind as EF  # noqa: E402
import e02_camerr as CE  # noqa: E402
import e02_trust as TR  # noqa: E402
import e02_load as LD  # noqa: E402
import e02_offload as OFF  # noqa: E402
import e02_stall as STL  # noqa: E402
import e02_trackwire as TKW  # noqa: E402  (the tracker process, frame saving, cpu.csv, fusion steering)
import e02_front as FR  # noqa: E402
import e02_unc as UNC  # noqa: E402
import e02_degrade as DG  # noqa: E402
import e02_turnwatch as TW  # noqa: E402
import e02_timeout as TO  # noqa: E402  (session timeouts: an unattended session always ends and uploads)
import e02_link as LNK  # noqa: E402  (commands that arrive: numbered, acknowledged, settings read back, link gauge)
LNK_SET_WAIT_S = LNK.SET_WAIT_MAX_S


def EX_provisional_camera(w, h, f=600.0, height_cm=260.0):
    """A stand-in camera before explore has fitted one: straight down from height_cm above the picture's centre (the
    tracker needs some floor scale to size a robot; explore only uses the pixels it gives back)."""
    import cameras as CMS
    return CMS.CameraModel([[f, 0, w / 2.0], [0, f, h / 2.0], [0, 0, 1]], [0, 0, 0, 0, 0], [math.pi, 0.0, 0.0],
                           [0.0, 0.0, height_cm], (w, h), name="provisional")

PAUSE_MS = 1000
FIX_HZ = 3.0
DRIVE_CMS_GUESS = 8.0
LEG_PAUSE_MS = 700        # e02_leg_pause_ms in every mode-0 run: still long enough for a camera fix (KC)
# 2026-10-24 review: at 700 ms, 160 of 242 KC fixes arrived after the robot had moved on. The pause is now set from
# the session's measured picture delay (leg_pause_ms()): the firmware's still check (e02_fix_still_ms 300) +
# KC_STILL_S of still pictures + the delay's 95th percentile + KC_SEND_MARGIN_S, between these bounds. All chosen
# (DOE: the share of stops with a fix that landed, and the time cost, at 1.0 / 1.4 / 2.0 s).
LEG_PAUSE_MIN_MS, LEG_PAUSE_MAX_MS = 1000, 2500
KC_STILL_S = 0.45           # kc_fix_from: 0.1 s after the stop + 0.3 s of still pictures, and a little
KC_SEND_MARGIN_S = 0.25     # the laptop's own work and the WiFi to the robot
KC_POOR_CELL_SD_FACTOR = 1.5   # chosen: the camera's sd in a poor cell (CAM.QualityMap); DOE with the tape marks
KC_HEAD_SD_DEG = 3.0
WEB_ASK_TIMEOUT_S = 15 * 60   # the Pi run: a question on the live page waits this long, then its default (chosen)        # chosen: the camera's heading from a straight stretch's end (DOE with tape marks)
TRANSIT_MAX_CM = 140      # e02_transit_max_cm: park spots are up to ~110 cm from the find's staging point
BACK_MAX_CM = 9.5         # firmware e02_back_max_cm is 10: backwards legs are split into pieces this long
TURN_DPS_GUESS = 45.0
ID_TURN_DEG = 40.0        # identify: turn this much on the spot and back (2026-10-22)
HOLD_CM = 6.0             # between legs the followed robot stands still: the camera only accepts it this near
CHECK_CM, CHECK_FRAC = 5.0, 0.35   # camera vs wheels: a go leg's distances may differ by this much...
ID_EXTRA_CM = 5.0                  # ...plus this on the first leg after a turn on the spot (its centre is rougher)
RECHECK_S = 3.5    # chosen (DOE note: one event, 2026-10-24 19:39, a ghost held 3 s; 3.5 s covers it with a margin and
                   # costs 3.5 s only on a disagreement; to test: count recheck rescues vs holds over the next sessions)
PARK_LEG_CM = 60.0        # parking: long legs on the robot's own wheels and gyro, the camera checks in between
STALE_S = 1.5             # a camera position older than this is not used to steer
ASIDE_PARK_CM = 15.0      # a robot put aside further than this from its park spot is driven there (own pose)
BLIND_CLEAR_CM = 35.0     # ... leaving the corner: first to this far from both boards
BLIND_PIECE_CM = 15.0     # driving blind (own wheels and gyro): drives no longer than this
BLIND_EDGE_CM = 15.0      # ... never to a point nearer an open edge than this
BLIND_RETRACE_LEGS = 6    # ... retracing: its last this many drives
BLOCKS_FIND_CM = 35.0     # a robot put aside this near the find's staging point blocks every find
RESCUE_EDGE_CM = 15.0     # a lost robot finds itself against the boards only if last seen this far in from the open edges (park spots: 17+)
SEARCH_LOOKOUTS = ((40.0, 40.0), (85.0, 40.0), (85.0, 85.0), (40.0, 85.0))   # (cm from the y-board, cm from the x-board)
SEARCH_CLEAR_CM = 25.0    # a lookout this near a robot we know of is skipped
SEARCH_STOP_AHEAD_CM = 4.0  # a searcher stopped by something in front: it is about this far ahead of its face
SWEEP_STEP_DEG = 30.0     # a finder robot turns in these steps, reading its front sensor after each
SWEEP_RANGE_CM = 70.0     # ... a reading nearer than this
SWEEP_GAP_CM = 8.0        # ... and this much shorter than the map says is something not on the map
SWEEP_NEAR_CM = 25.0      # the camera then looks for the lost robot's turn within this of it
FRONT_OFFSET_CM = FR.DEFAULT_OFFSET_CM                  # front sensor reading minus the map's distance: measured, 115
FRONT_TOL_CM = FR.TOL_SD * FR.DEFAULT_SD_CM             # readings of 2026-10-23/24 (was 0 / 8 cm, chosen); with
                          # --no-front-learn only: by default each robot's are learned every session (e02_front.py)
FRONT_STEER_FACTOR = 1.5  # a disagreement this many tolerances, with a measured heading, makes the runner find it again
FRONT_LEARN_MIN = 8       # --front-learn: readings before the learned offset and tolerance are used
FRONT_LEARN_MIN_TOL = 3.0
FRONT_MIN_COS = 0.7       # the front-sensor check only counts boards met within 45 deg of head-on
# Item 4 (Nick, 2026-10-23): confidence pauses. The robot's own position uncertainty (1 sd, cm) grows with what it
# has driven and turned since its last camera fix or find; before a drive that would take it past the threshold it
# stops and takes a fresh camera fix. Model constants chosen from session 1 (calibrated runs ~1.4 cm after ~200 cm):
# wheel scale ~1% left after self-calibration, heading 0.5 deg + 0.5% of what it turned. The threshold: Nick 3 cm,
# to be settled by a DOE (docs/experiments/DOE-confidence.md).
# 2026-10-25 (Nick: "create a calibration run that establishes this on its own, without simulators"): the four model
# numbers are now LEARNED every session from the robot's own camera corrections (e02_unc.py: earlier sessions', then
# this session's own from 30 corrections on, reported when they differ), saved in logs/unc_learned.json. These are
# only where a robot with no history starts: the logs DOE's refit (turning 0.014 per degree, was 0.005; 673
# corrections, docs/build-reports/e02-oct25-logs-doe.md row 1). --no-unc-learn keeps them fixed.
CONF_PAUSE_CM = 3.0
UNC_BASE_CM, UNC_PER_CM, UNC_DEG, UNC_DEG_PER_DEG = UNC.DEFAULT
# Item 5 (Nick, 2026-10-23: option a): pictures older than this are "late": the robot drives on its own wheels and
# gyro, and the front sensor reading a board square on corrects its position along that board's axis.
LATE_PICTURE_S = 1.0
FRONT_CHECK_RANGE_CM = 50.0  # the front-sensor check only counts things nearer than this
FRONT_CHECK_CM = 6.0      # ... and says so when the sensor and the map differ by more
EXPECT_TURN_CM = 7.0       # a robot turning on its spot stays within this of where it stood (camera's view)
EXPECT_GO_EXTRA_CM = 8.0   # ... a drive of d cm within d + this
HEADING_CLEAR_CM = 20.0   # the heading drive (10 cm) only with at least this much room in front
ID_UNKNOWN_SCALE = 1.7    # a robot found by a turn on its spot: its floor is unknown this many robot widths round
AUTO_TRIES = 3            # a lost or stuck robot tries this many times on its own before Nick is called
BACK_OFF_CM = 8.0         # a leg stopped by its front sensor backs this far before turning away (2026-10-22 loop)
EDGE_TURN_CM = 9.0        # no turn on the spot closer than this to an open edge (the wheels reach it first); since the
# 15:45 fixes drive_to obeys it too (it moves straight away first). DOE note: yes, on the board with the crawl DOE:
# turns on the spot at 6 / 9 / 12 cm, camera-measured body sweep (doe-backlog.md "No turns near an open edge")
# --- 2026-10-25 15:45 session fixes (docs/build-reports/e02-oct25-1545-fixes.md). The dark strips along the bottom
# AND the right edge are painter's tape on the plywood (Nick, 18:46 for the bottom; 2026-10-26 for both): surface, not
# a drop. At the bottom the drop is within about 1-2 cm of the stored line; at the right edge the 21:14 video puts it
# about 4 cm OUTSIDE the stored line (the tape's outer side). Every distance below is from the stored lines, so at the
# right edge the rules keep about 4 cm more than they say (a margin, not a hazard).
EXPLORE_TURN_ROOM_CM = 5.0    # 2026-10-26 runner fixes, item 3 (Nick: no turn on the spot "at or past an edge"): while
# exploring (camera only roughly placed, the guard's frame) a robot whose centre the camera puts within this of an open
# edge's line, or past it, is AT the edge: its 9.6 cm wide body reaches the line (half width 4.8 cm). Not EDGE_TURN_CM
# (9 cm): dry run D3, Left driving along the bottom edge 7 cm from it, could never turn (straight moves along the edge
# do not get it further out). Its own edge stop is the other trigger. DOE note: yes, with EDGE_TURN_CM's row (turns
# on the spot at 4 / 6 / 9 cm from the edge, body sweep on camera)
EDGE_AWAY_MARGIN_CM = 1.5     # chosen: move away until the centre is this much past EDGE_TURN_CM (the camera's noise
# at an edge, about 1 cm). DOE note: no, a margin on a limit that has its own DOE row
EDGE_AWAY_MIN_COS = 0.34      # chosen: straight driving only moves it away if it faces within 70 deg of the edge's
# normal (either way); along the edge it turns slowly instead (logged edge_turns_along). DOE note: no, geometry
EDGE_AWAY_MAX_CM = 12.0       # chosen: at most this far in one move-away leg (then planned again). DOE note: no
HEADING_CRAWL_ROOM_CM = 20.0  # heading unknown and the centre within this of an open edge: no blind drive, a crawl. The
# 15:45 analysis said 15; the 19:02 dry run showed a 10 cm heading drive from 17 cm ending 9 cm from the edge, so it
# is the 10 cm drive + the watchdog's 9 cm + 1. DOE note: yes, the crawl DOE (doe-backlog.md "Heading-drive crawl
# near an edge")
HEADING_CRAWL_CM = 3.0        # Nick, 18:46: "about 3 cm" a crawl step; at most 2 steps until the camera has seen 3 cm
HEADING_CRAWL_CMS = 2.0       # chosen: slow, as explore's creep (EXPLORE_CREEP_CMS). Same DOE row
HEADING_CRAWL_BACK_CM = 6.0   # the 15:45 analysis: an edge stop on the crawl backs straight out this far (it came that
# way, so it is clear; 6 cm also measures the heading on the way). Same DOE row
TURN_SLOW_ROOM_CM = 20.0      # chosen: within this of an open edge turns go slower (Nick: "more cautious around turns",
# not slower everywhere). DOE note: yes, with the turn-near-edge DOE row: body sweep and gyro error against turn speed
TURN_SLOW_RPM = 20.0          # chosen: half the firmware's turn_max_rpm (40). Same DOE row
TURN_FAST_RPM = 40.0          # params.h turn_max_rpm, put back away from the edges
TURN_SLOW_TIMEOUT_MS = 9000   # a slow turn gets this long (params.h turn_timeout_ms 5000 at full speed: a 180 deg turn
# at half speed takes about 8 s on the real robots, 45 deg/s at full speed). Derived from the speed, no DOE
TURN_TIMEOUT_MS = 5000        # params.h turn_timeout_ms, put back
TURN_CONFIRM_ROOM_CM = 25.0   # chosen: after a turn this near an open edge, the first straight drive is a short check
# (HEADING_CHECK_CM) the camera measures before the rest. In the open it drives at full length. DOE note: yes, logs:
# how far the first drive after a turn went off its heading in every session, by distance to the edge
CRAWL_MIN_ROOM_CM = 5.5       # chosen: no heading crawl with the centre nearer an open edge than this: the line
# sensors (3.5 cm ahead of the axle) could already be over the drop and the guard would learn it as floor (the
# 2026-10-22 fall); Nick is called. DOE note: no, geometry (3.5 cm + 2 cm of camera error)
CRAWL_HEADING_MIN_CM = 1.5    # chosen: a crawl's heading counts from 1.5 cm of straight camera track (a straight-line
# fit through every locked picture, rms under 0.6 cm). Dry runs 19:11-19:28: near the bottom edge the simulator's
# camera saw only 33-40 % of the crawl (2.0-2.3 of 6.0 cm) but in the right direction (within 2 deg) whenever it
# stayed locked. DOE note: yes, with the crawl DOE row (heading error against camera distance, on the real board)
CRAWL_LOCKED_SHARE = 0.8      # chosen: ... and only if the tracker was locked on in this share of the crawl's pictures
# (19:28: locked in 32 of 56, the direction 25 deg off; 19:18: detector only, 165 deg off). Same DOE row
CRAWL_CAMERA_SHARE = 0.25     # chosen: a crawl's camera track counts only if it saw at least this share of what the
# wheels drove (a backstop: the simulator's camera saw 33-49 % near the bottom edge, right direction). DOE note: yes,
# the crawl DOE row
FOUND_SETTLE_S = 1.5          # chosen: after a find the watchdog's distance check waits this long for the tracker to
# lock on. DOE note: no, a settling window (the tracker's own lock-on time)
CRAWL_JUMP_CMS = 8.0          # chosen: while crawling (2 cm/s) a camera position moving faster than 4x that is the
# tracker settling, not the robot. DOE note: no, a multiple of the crawl speed
CRAWL_WATCH_EDGE_CM = 5.0     # chosen: while a robot crawls towards an open edge on purpose (the heading crawl,
# explore's crawls), the watchdog stops it only this near (its front then at the drop): the edge sensors stop it
# first (the bottom zone looks about 9 cm ahead). DOE note: no, a backstop behind the edge sensors
HEADING_CHECK_CM = 4.0        # chosen: the check drive (ST.MIN_HEADING_CM 3 cm measures a heading; 1 cm of margin)
TURN_SPIN_DRIFT_CM = 2.5      # chosen: a turn on its spot whose camera centre moved this much towards an open edge,
# inside the border, holds like a drive into the border (the top's centre swings about 1.5 cm on a turn). DOE note:
# yes, logs: centre drift during healthy turns (track.csv) against Right's 15:45 spins
EDGE_BACKOUT_S = 30.0         # (until 2026-10-26: a backwards drive this soon after the robot's own edge stop was a
# back-out. Now "right after" = no turn or forward drive has run since, however long a hold took: backing_out)
FIX_HELD_AFTER_GO_S = 5.0     # chosen: this soon after a drive, a "held" tracker spot is not sent as a camera fix until
# the camera has really seen the robot again (15:45: a held spot 15 cm off was sent to Left). DOE note: no
FIX_HELD_WAIT_S = 1.5         # ... waiting at most this long for a real sighting (then no fix). DOE note: no
ASIDE_ROUTE_MARGIN_CM = 3.0   # chosen: a route passing a set-aside robot nearer than the watchdog's parked-robot
# distance plus this is not run (15:45: 4 square runs cut 14-17 cm from parked Right). DOE note: no, a margin on
# PL.WATCH_ROBOT_CM
ALERT_REPEAT_S = 30           # the live page sounds again this often while a hold or help call waits (chosen: Nick
# missed two 10-minute help calls). DOE note: no, Nick's time
FIND_TIMEOUT_S = 150
E02_TIMEOUT_S = {0: 150, 1: 400}
BATTERY_WARN, BATTERY_STOP = 40, 15
LEVELS_EVERY_S = 300.0      # chosen: refit the battery and the charge levels at most every 5 min (the battery reads in
                            # whole %; about 1 % per 4 min while standing). No DOE: it only sets how fresh the display is
EXPECTED_FW = "2026-10-26a"
OK_FW = ("2026-10-22b", "2026-10-25a", "2026-10-25b", "2026-10-25c", "2026-10-26a")   # 25a adds the robot's own virtual
                                                        # border and `sleep`; 25b the back switches and the reverse find;
                                                        # 25c refuses go / turn legs before a compass check; 26a numbered,
                                                        # acknowledged commands (e02_link.py), `dup` before any refusal,
                                                        # and the tilt stop; 22b still runs (plain commands, no tilt stop)
BORDER_FW = ("2026-10-25a", "2026-10-25b", "2026-10-25c", "2026-10-26a")   # the robot's own virtual border and `sleep`
# --switches-back (Nick, 2026-10-25: "one attachment in total with switches in the back"): these too, sent and read
# back with the safety settings and again before every find (a robot switched off and on loses them; it would then
# find nose first, which still works). The find then backs into each board (firmware 2026-10-25b, e02.h REVERSE
# FIND). e02_back_face_cm: hardware/dock/build_attachment_v3.py's rear bar (a design number, to be tape-measured).
# The boards stand on the cement pressed against the plywood's side (Nick, 2026-10-25), so a board's line is the
# plywood's edge: touching it, the rear caster (4.1 cm behind the axle) is 2 cm inside the edge. The firmware's
# backing cap (e02_find_keep_cm 2) is left as it is: the look before backing is what keeps a robot off an edge where
# a board was taken away (firmware e02.h REVERSE FIND; build report e02-oct25-backing-unc.md, item B).
BACK_SETTINGS = (("sw_on", 1.0), ("sw_back", 1.0), ("e02_find_reverse", 1.0), ("e02_back_face_cm", 6.10))
BACK_FW = ("2026-10-25b", "2026-10-25c", "2026-10-26a")
TILT_FW = ("2026-10-26a",)    # firmware that stops itself on a tilt (motion.h TILT) and refuses legs while tilted
# Backups (Nick, 2026-10-23: "build in backup plans in case our session goes poorly ... get as much data as possible
# before stopping for a rebuild"). Chosen numbers; a DOE would not help much here (they trade data for Nick's time).
HEADING_LONG_ROOM_CM = 30.0   # a 15 cm heading drive only this far (or more) from an open edge
HELP_DROP_N = 3           # help calls for one robot during the runs: then it is put aside, not Nick called again
COND_FAIL_N = 3           # failed runs of a camera-dependent condition (K, KC): its remaining runs are put aside
# The edge guard (both cues, Nick 2026-10-22 evening: "essential if we're going to be mapping by feel alone") and
# the bare body. safe_edge is not saved on the robot: set after every power-on.
# Line cue, edge_thr 150 (chosen; a DOE would not add much: the gap is wide): on this board, driving, the line
#   sensors read 52-84 (all E02 sessions' RAW, 2026-10-21/22); at the two real edge trips (Left, 22nd evening) 241
#   and 573. Plywood + 66 or more is never reached on plywood; both trips were well past it.
# Bottom-zone cue, edge_bot_cm 11 (chosen as a backup): driving on plywood the bottom zone read at most 10.3 cm in
#   every real log (8.8 on this board; each robot has its own level, 7.2-8.8), and at the two line trips only 9.0 and
#   9.7, so it fires later than the line cue, if the line sensors miss. What it reads further past the edge is not
#   measured: every EDGE record now says which cue fired first and its reading (tests-to-run: cliff sense). A DOE
#   would be sensible for this one (limit vs false stops vs how far past the edge), once the edge test has data.
# The camera watchdog is a third, independent guard.
SAFETY_SETTINGS = (("safe_edge", 3.0), ("edge_dir", 0.0), ("edge_thr", 150.0), ("edge_n", 3.0), ("edge_bot_cm", 11.0),
                   ("bumper_cm", 0.0), ("e02_face_cm", 0.0), ("e02_back_max_cm", 10.0),
                   ("e02_cms", 10.0))   # back_max and cms: undo explore's raise and crawl
SAFETY_CMDS = ["set %s %g" % kv for kv in SAFETY_SETTINGS]
DEFAULT_CALIB = os.path.join(HERE, "data", "overhead_floorcal_roam1.json")   # roam1 board corners, camera unmoved
def stretch_rms_against(obs, fitted, prior):
    """(rms cm, rms deg) of these stretches against fixed values (the pooled fit's, else the prior)."""
    w = fitted.get("wheel_scale") or prior["wheel_scale"]
    lens = [(o["cam_len"], o["bel_len"]) for o in obs if o.get("cam_len") and o.get("bel_len")]
    # robust (median-based) so one stretch where the tracker jumped does not count against the wheels
    rc = TR.robust_sd([c - w * b for c, b in lens]) if lens else None
    g = fitted.get("gyro_scale") or prior["gyro_scale"]
    d = fitted.get("gyro_drift_dps") if fitted.get("gyro_drift_dps") is not None else prior["gyro_drift_dps"]
    t = FIT.fit_turns(obs, prior=dict(gyro_scale=g, gyro_drift_dps=d),
                      prior_sd=dict(gyro_scale=1e-6, gyro_drift_dps=1e-6))
    return rc, (t["rms_deg"] if t else None)


def run_scales(obs):
    """[(run, wheel scale, wheels' length cm, [(camera cm, wheels cm)])] per run: the camera's length over the wheels'
    length summed over that run's stretches (two or more), and the stretches themselves, for e02_trust's runs check
    (2026-10-25: a wheel that counts wrongly by a different amount every run shows as runs that disagree beyond what the
    camera's own scatter about each run explains, with no other robot needed)."""
    per = {}
    for o in obs:
        if o.get("cam_len") and o.get("bel_len"):
            per.setdefault(o.get("run"), []).append((o["cam_len"], o["bel_len"]))
    out = []
    for run, lens in per.items():
        b = sum(x[1] for x in lens)
        if len(lens) >= 2 and b > 0:
            out.append((run, sum(x[0] for x in lens) / b, b, lens))
    return out


def trust_path(a):
    """Where the learned sensor trust lives (kept between sessions, never in git): logs/sensor_trust.json; dry runs
    and degraded tests keep their own, so they never touch the real robots' trust."""
    if getattr(a, "trust_file", None):
        return a.trust_file
    slug = re.sub(r"[^A-Za-z0-9]+", "_", getattr(a, "degrade", "") or "").strip("_")
    name = "sensor_trust%s%s.json" % ("-dryrun" if getattr(a, "dry_run", False) else "",
                                      ("-degraded-" + slug) if slug else "")
    return os.path.join(REPO, "logs", name)


def trust_base_path(a):
    """A degraded test starts from the trust the system has learned without it (Nick: swap a sensor on a trained
    system), and never writes back there (2026-10-24 review)."""
    if not getattr(a, "degrade", ""):
        return None
    return os.path.join(REPO, "logs", "sensor_trust%s.json" % ("-dryrun" if getattr(a, "dry_run", False) else ""))


# The virtual border (Nick, 2026-10-24 evening, after Left and Center went over the open edge in the Pi plan's
# stage 3): a robot the camera sees within BORDER_CM of an open edge (its centre), moving towards it, stops EVERY robot
# and the session waits for Nick (hold_for_nick). Every session. 15 cm: the Pi's worst picture age (0.56 s) at 10 cm/s
# is about 6 cm, braking about 1 cm, the robot's half length 5 cm, so it stops before a wheel reaches the edge.
# Chosen, not measured: a DOE should set it (drive at the line at 4 and 10 cm/s a few times, measure the overshoot).
# Exempt: a forward crawl that is looking for the edge on purpose (explore's rays and probes, the edge find): the
# firmware's edge guard stops those, forwards; anything backwards, or at speed, is never exempt.
BORDER_CM = 15.0
# A camera that looks bad shrinks the working area instead (Nick's question, 2026-10-24 night: shrink the area, not
# loosen the border): the border moves in by BORDER_EXTRA_SIGMA x the camera's noise above healthy, at most
# BORDER_EXTRA_MAX_CM, while the camera is flagged (e02_trust), and back when it looks healthy again.
BORDER_EXTRA_SIGMA = 2.0      # chosen (DOE note: 2 sd of the extra noise keeps ~98% of single readings inside; the
                              # simulator grid's camera:noise=2/4 sessions are the test, more real data would set it)
BORDER_EXTRA_MAX_CM = 10.0    # chosen: a 25 cm border still leaves the L and square routes room on the 120 cm board
# The robot's own border (firmware 2026-10-25a) is the backstop for when the camera can't see or can't be trusted,
# not a second copy of the camera's line: at 15 cm it sat inside the parking spots' band and, with a noisy camera
# (2026-10-25 simulator, camera noise 2 cm), held the session 5 times in 5 minutes during setup alone.
FW_BORDER_CM = 10.0           # chosen (DOE note: a robot's front is 5 cm ahead of its centre, braking about 1 cm, its own
                              # pose drifts a few cm between camera fixes; a DOE driving at the line uncalibrated, at
                              # 4 and 10 cm/s, would set it). Fixed: a bad camera moves the camera's line and the routes
                              # in (border_cm), never this one
# Home mode (--home; Nick, 2026-10-25: "we can take more risk when I'm home", the goal being to drop the virtual border
# eventually): the camera's border and the robots' own both move closer to the edge; a crossing still stops every
# robot and asks. Away sessions (no --home) keep BORDER_CM / FW_BORDER_CM.
HOME_BORDER_CM = 8.0          # chosen (DOE note: front 5 cm ahead of the centre, braking ~1 cm, picture age ~0.2 s at
                              # 10 cm/s = 2 cm; the drive-at-the-line DOE at 4 and 10 cm/s would set it)
HOME_FW_BORDER_CM = 6.0       # chosen: the robots' own stays below the camera's, as away (15 / 10); its front 1 cm from
                              # the edge, where the forward edge sensors also stop it
PARK_OPEN_MAX_CM = 22.0       # while the camera looks bad, park spots move in to border_cm() but no further than this
                              # from the open edges: at 22 the square route still passes 22 cm from the moved spot's
                              # centre (at 25 it would be 19.2, near PARK_CLEAR_CM 18). Chosen from the nominal paths;
                              # the uncalibrated-drift simulator test (test_e02_runner) is the check
BORDER_TOWARDS_CMS = 2.0      # chosen: moving towards the edge faster than this counts (still or away is allowed) ...
BORDER_TOWARDS_FRAC = 0.5     # ... and mostly towards it (within 60 deg of straight at it): driving along the edge is
                              # allowed (dry run 2026-10-24, a robot put down 10 cm from the edge backed along it in
                              # its room check, 1-2 cm/s sideways drift, and held the session)
BORDER_JITTER_CM = 1.5       # 2026-10-26 runner fixes, item 2: the border check's "moving towards / further out" needs the
                              # robot's centre to come this much nearer the edge over its 0.6 s window, by a straight
                              # line fitted through the window's pictures (line_ends). Measured: a robot standing still
                              # for 4 h (21:14 overnight hold, 130,573 windows of 0.6 s, 10 pictures a second): p99.9
                              # 1.17 cm, p99.99 1.40, max 1.52 (single first-to-last pictures, as before: max 1.93; the
                              # 21:14 false holds were 0.8 and 1.0). A real approach at 4 cm/s shows 2.4 cm. DOE note:
                              # set from data (analyses_oct26/camera_noise/still_jitter.py); a moving robot's wobble is
                              # not measured (the drive-at-the-line DOE would show whether 1.5 cm hides a slow approach)
BORDER_JUMP_CMS = 50.0        # chosen: faster than this is the tracker jumping to a new spot, not the robot (it drives
                              # at most ~20 cm/s); replaying the 2026-10-24 Pi plan's tracks, one such jump would have held
# The speed guard (2026-10-25 runner fixes, part 2b; at 15:10-16:00 the pictures handled a second fell 24 -> 15 -> 10
# and a robot crossed home's 8 cm border). The camera's lag = a picture's age when its position is ready + one picture
# interval (1 / pictures a second): how long a robot drives before the runner can see where it is. The borders were
# sized for a lag budget (HOME_BORDER_CM's note: ~0.2 s at 10 cm/s = 2 cm; BORDER_CM's: the Pi's worst age 0.56 s).
# Above the budget, each robot's border moves out by how far IT drives in the extra lag (its speed x (lag - budget)):
# a robot driving towards an open edge inside that wider band is stopped (no hold: nothing is wrong with the robot),
# and no robot in the band is sent a new move until the lag is back within budget (at most SPEED_WAIT_MAX_S).
SPEED_LAG_BUDGET_HOME_S = 0.2   # from HOME_BORDER_CM's own note (not new). 15:10's baseline lag was 0.18 s (age 0.123 +
                                # 1/16.7): inside it. DOE note: the drive-at-the-line DOE (doe-backlog) run at three
                                # camera rates (--dry-slow-camera, or a --load on the Pi) would set both budgets
SPEED_LAG_BUDGET_AWAY_S = 0.56  # from BORDER_CM's note (the Pi's worst picture age then); same DOE
SPEED_GUARD_MIN_CM = 0.5        # chosen: a wider band than this switches the guard on (below it is camera noise, sd
                                # ~0.3 cm still). No DOE: a noise floor
SPEED_GUARD_MAX_CM = 15.0       # chosen: the band never grows past this (10 cm/s x 1.5 s of lag); a camera slower than
                                # that is the lost check's job (LOST_DRIVE_S). DOE note: no, a cap on a formula
SPEED_WAIT_MAX_S = 60.0         # chosen: a robot in the band waits at most this long for the camera before its move
                                # goes anyway (the in-motion stop still guards it). 15:10's slow spells lasted 8-36 s.
                                # DOE note: no; set from the slow spells' lengths once the fits are out of the way
SPEED_RECOVER_ROWS = 2          # chosen: two perf rows (seconds) in budget in a row before moves start again (one good
                                # second inside a slow spell is common: 15:10 perf.csv). DOE note: no, a debounce
# A robot driving a leg that the camera has no position for this long is lost: everything stops and Nick is asked
# (Nick, 2026-10-24: "stop and wait"; in stage 3 the tracker stayed on Left's old spot while it drove). Chosen.
LOST_DRIVE_S = 1.5
LINK_PAUSE_MAX_S = 30 * 60.0  # a link pause this long ends the session like an unanswered hold (Nick's 30 min answer
                              # timeout, e02_timeout.ANSWER_TIMEOUT_MIN). DOE note: no, a convenience setting
LOST_START_S = 1.0            # a leg's first second: the firmware's still check and the start (it may not move yet)
STUCK_MIN_S = 3.0             # item 4 ("for a few seconds", Nick): the camera looks back this long. DOE note: yes, with
                              # the tilt bench DOE (a robot held by a wheel off a step): 2 / 3 / 5 s against false stops
                              # on a slow camera
STUCK_WHEELS_CM = 5.0         # item 4: the wheels should have taken it at least this far (21:14 report: 5 cm). Chosen;
                              # same DOE
STUCK_CAMERA_CM = 2.0         # item 4: ... and the camera saw it move less than this. From data: a robot standing still
                              # moves 1.47 cm at most between the medians of 3 pictures (4 h, 21:14 overnight hold); a
                              # robot driving 5 cm moves ~5. DOE note: set from data
TILT_STOP_DEG = 5.0           # item 4: the robot's tilt (RAW pitch / roll) this far from its own level stops it (as the
                              # firmware's tilt_stop_deg). Real RAW (5 sessions, 210,000 rows on plywood, driving
                              # included): within 4.1 deg of each robot's level, 99.9 % within 3.0; Left hanging: 10.2.
                              # DOE note: yes, the bench tilt DOE (a wheel off a 1 / 2 / 3 cm step; nose over the edge at
                              # 2 / 4 / 6 cm) would set it
TILT_MS = 300.0               # item 4: for this long (the one 4.1 deg reading lasted 17 ms). Chosen; same DOE
LEG_OVER_MARGIN_S = 2.0       # chosen: braking, slip and speeding up (a 10 cm leg at 10 cm/s takes ~1.5 s, not 1.0). No
                              # DOE: a margin; the leg times in leg.csv would show the spread if it matters
TILT_KEEP = 3000              # RAW rows of tilt kept per robot for its level (about 2.5 min of RAW at 20 a second)
LEG_START_AFTER_ACK_S = 0.35  # an idle leg starts this long after the robot gets it (edge_learn_ms 300 + 50, e02.h
                              # idleTick); from the firmware, no DOE
LOST_NO_START_S = 3.0         # older firmware (no answers) with the 'started' record lost: counted from the send + this
                              # (chosen: 3 x the 99.9 % reply time with margin; no DOE, a fallback)
LOST_FROZEN_S = 3.0           # chosen: a go leg this old ...
LOST_FROZEN_CM = 10.0         # ... that should have gone at least this far ...
LOST_FROZEN_FRAC = 0.25       # ... and the camera's spot moved less than this share of it: the camera lost it
HOLD_DRY_ANSWER_S = 5.0
HOLD_RESTOP_S = 2.0           # chosen: while a hold waits, every robot is told to stop again this often       # a dry run with nobody to ask: "carry on" after this long (said loudly)
HOLD_MOVE_CMDS = ("go ", "turn ", "run ", "e02 ", "calibrate", "identify", "dock", "follow")
RESCUE_KINDS = ("lost", "border", "stuck", "tilt", "find")   # holds about one robot: no answer -> it rescues itself
                                                             # (item 7); anything else still ends the session


def leg_over_s(val, cms):
    """How long after its start a go leg of val cm at cms cm/s must be over (its start + the drive + LEG_OVER_MARGIN_S);
    after that a robot with no end record has finished and its record was lost (2026-10-26 dry run D2: a lost 'done'
    record made a parked robot look stuck)."""
    return LOST_START_S + abs(val or 0.0) / max(0.5, float(cms or EXPLORE_FAST_CMS)) + LEG_OVER_MARGIN_S


def line_ends(smp):
    """The start and end points ((x0, y0), (x1, y1)) of a least-squares straight line through (t, x, y, ...) samples,
    fitted on t: a window's move with the camera's wobble averaged out (border check, 2026-10-26)."""
    ts = [q[0] for q in smp]
    tm = sum(ts) / len(ts)
    sxx = sum((t - tm) ** 2 for t in ts)
    out = []
    for k in (1, 2):
        vs = [q[k] for q in smp]
        vm = sum(vs) / len(vs)
        b = sum((t - tm) * (v - vm) for t, v in zip(ts, vs)) / sxx if sxx > 0 else 0.0
        out.append((vm + b * (ts[0] - tm), vm + b * (ts[-1] - tm)))
    return (out[0][0], out[1][0]), (out[0][1], out[1][1])


def num(v):
    try:
        f = float(v)
        return f if math.isfinite(f) else None
    except (TypeError, ValueError):
        return None


def leg_delta(start, fin):
    """An idle leg's own move (x forward, y left, th turned) from its 'started' and final LEG records. A robot
    with no pose yet starts every idle leg at (0, 0, 0), so the final pose is already the move; one whose pose
    was set by camera fixes (a session before, not switched off) reports its board pose (2026-10-24 Pi plan,
    stage 3). Returns (None, None, None) if the final pose is missing; the final pose as is if the start is."""
    x1, y1, t1 = (num(fin.get("x_cm")), num(fin.get("y_cm")), num(fin.get("th_deg")))
    if None in (x1, y1, t1):
        return None, None, None
    x0, y0, t0 = (num((start or {}).get("x_cm")), num((start or {}).get("y_cm")), num((start or {}).get("th_deg")))
    if None in (x0, y0, t0):
        return x1, y1, t1
    dx, dy = x1 - x0, y1 - y0
    c, s = math.cos(math.radians(t0)), math.sin(math.radians(t0))
    return dx * c + dy * s, -dx * s + dy * c, ST.wrap180(t1 - t0)


def hms(s):
    s = max(0, int(s))
    return "%d:%02d" % (s // 3600, (s % 3600) // 60)


# =============================================================================== robot side listener
class Listener:
    """Everything the robots send that the runner needs, read by column name (the firmware lane may add
    columns or change their order)."""

    def __init__(self, session):
        self.s = session
        self.cv = threading.Condition()
        self.raw = defaultdict(list)          # run id -> [dict(t_recv, ms, x_cm, y_cm, th_deg, yaw_deg)]
        self.recs = defaultdict(list)         # type -> [(t_recv, robot, dict)]
        self.runinfo = {}                     # run id -> dict
        self.battery = {}                     # robot -> (t, %)
        self.firmware = {}
        self.msgs = []                        # (t, robot, text)
        self.lab = None                       # e02_live.LiveLab, fed every RAW row
        self.tilt = {}                        # robot -> deque of (t_recv, pitch, roll) from RAW (item 4)

    def __call__(self, name, kind, rtype, vals):
        t = time.time()
        with self.cv:
            if kind == "M":
                text = vals if isinstance(vals, str) else ",".join(vals)
                self.msgs.append((t, name, text))
                if len(self.msgs) > 2000:
                    del self.msgs[:1000]
                low = text.lower()
                if "battery" in low:
                    k = low.find("battery")
                    digits = "".join(ch for ch in low[k + 7:k + 13] if ch.isdigit())
                    if digits:
                        self.battery[name] = (t, int(digits))
                if "(firmware " in text:
                    k = text.find("(firmware ")
                    self.firmware[name] = text[k + 10:text.find(")", k)]
            elif kind == "D":
                cols = self.s.headers.get(rtype)
                rec = dict(zip(cols, vals)) if cols and len(cols) == len(vals) else {}
                if rtype == "RAW" and rec and self.lab is not None:
                    try:
                        self.lab.on_raw(name, rec, t)
                    except Exception:                 # the live lab is a display: never break the listener
                        pass
                if rtype == "RAW" and rec:
                    pr = (num(rec.get("pitch_deg")), num(rec.get("roll_deg")))
                    if None not in pr:
                        self.tilt.setdefault(name, deque(maxlen=TILT_KEEP)).append((t, pr[0], pr[1]))
                    run = rec.get("run")
                    lst = self.raw[run]
                    if len(lst) < 40000:
                        lst.append(dict(t_recv=t, ms=num(rec.get("ms")), x_cm=num(rec.get("x_cm")),
                                        y_cm=num(rec.get("y_cm")), th_deg=num(rec.get("th_deg")),
                                        yaw_deg=num(rec.get("yaw_deg"))))
                elif rtype == "RUNINFO" and rec:
                    self.runinfo[rec.get("run")] = rec
                    b = num(rec.get("battery"))
                    if b is not None:
                        self.battery[name] = (t, int(b))
                elif rtype == "RUNEND" and rec:
                    b = num(rec.get("battery"))
                    if b is not None:
                        self.battery[name] = (t, int(b))
                elif rtype in ("BUMP", "FIND", "E02", "LEG", "GYRO", "CORR", "BORDER") and rec:
                    self.recs[rtype].append((t, name, rec))
            self.cv.notify_all()

    def tilt_dev(self, robot, hold_s, now=None):
        """How far (deg) the robot's tilt has been from its own level for the last hold_s at least (the smallest
        deviation over that span), or None (no RAW rows that recent). Its level: the median pitch and roll of its RAW
        rows kept (TILT_KEEP; a robot's level differs, -1.5 to +1 deg in the real logs)."""
        now = time.time() if now is None else now
        with self.cv:
            d = self.tilt.get(robot)
            if not d or len(d) < 20 or now - d[-1][0] > 1.0:
                return None
            rows = list(d)
        recent = [r for r in rows if r[0] >= now - hold_s - 0.05]
        if len(recent) < 2 or recent[-1][0] - recent[0][0] < 0.5 * hold_s:
            return None
        p0 = sorted(r[1] for r in rows)[len(rows) // 2]
        r0 = sorted(r[2] for r in rows)[len(rows) // 2]
        return min(max(abs(r[1] - p0), abs(r[2] - r0)) for r in recent)

    def latest(self, rtype, robot, after=0.0, run=None):
        with self.cv:
            for t, n, rec in reversed(self.recs[rtype]):
                if n == robot and t >= after and (run is None or rec.get("run") == run):
                    return t, rec
        return None

    def wait_rec(self, rtype, robot, after, timeout, run=None, poll=None):
        end = time.time() + timeout
        while time.time() < end:
            r = self.latest(rtype, robot, after, run)
            if r:
                return r
            with self.cv:
                self.cv.wait(0.1)
            if poll and poll():
                return None
        return None

    def leg_final(self, robot, run, lid):
        """The final LEG record (done / failed / refused / dup) of leg id `lid` in `run`, or None. A done or failed
        record wins over a dup or refused one for the same id, whatever came last (2026-10-26 explore reliability:
        a resent leg's `dup` answer arriving after its `done` hid the done, and explore recorded a 175 deg turn or an
        11 cm back-out as no move at all; 4-8 legs a run at 10 % loss)."""
        other = None
        with self.cv:
            for t, n, rec in reversed(self.recs["LEG"]):
                state = rec.get("state") or rec.get("result")
                if n == robot and rec.get("id") == str(lid) and rec.get("run") in (run, "-") and \
                        state in ("done", "failed", "refused", "dup"):
                    if state in ("done", "failed"):
                        return dict(rec, state=state)
                    if other is None:
                        other = dict(rec, state=state)
        return other

    def leg_start(self, robot, run, lid):
        """The 'started' LEG record of leg id `lid` (the robot's pose as the leg began), or None."""
        with self.cv:
            for t, n, rec in reversed(self.recs["LEG"]):
                state = rec.get("state") or rec.get("result")
                if n == robot and rec.get("id") == str(lid) and rec.get("run") in (run, "-") and state == "started":
                    return dict(rec, state=state)
        return None

    def wait_leg(self, robot, run, lid, after, timeout, stop=None):
        end = time.time() + timeout
        while time.time() < end:
            r = self.leg_final(robot, run, lid)
            if r:
                return r
            with self.cv:
                self.cv.wait(0.1)
            if stop and stop():
                return self.leg_final(robot, run, lid)
        return None

    def got_params(self, robot, since):
        """{name: value} from `get` answers ("M,name = value ...") since a time."""
        out = {}
        with self.cv:
            for t, n, m in self.msgs:
                if n != robot or t < since or " = " not in m:
                    continue
                k, _, rest = m.partition(" = ")
                k = k.strip().split(",")[-1].strip()
                try:
                    out[k] = float(rest.split()[0])
                except (ValueError, IndexError):
                    pass
        return out

    def said(self, robot, since, words):
        with self.cv:
            return [m for t, n, m in self.msgs if n == robot and t >= since and any(w in m.lower() for w in words)]


# =============================================================================== the runner
EXPLORE_SEEN_CONTACTS = True   # 2026-10-26 explore reliability: robots left out of the camera fit still give their contacts,
                               # placed by the fitted camera (e02_explore.contacts_seen)
EXPLORE_LOST_AS_TOLD = True    # 2026-10-26 explore reliability: a leg whose record never arrived is taken as told (marked "lost")
EXPLORE_SIZE_CHECK_PCT = 6.0   # 2026-10-26 explore reliability: explore's board is not trusted when a side is more than this
                               # far off the plywood's known 122 cm (0 = no check). Chosen: the before batch's passes were
                               # 0-5 % off but one (8.8 %); the after batch's wrong pass 15 %. DOE note: yes, simulator: the
                               # batch's sizes against 4 / 6 / 8 %, wrong boards let through against good ones refused
EXPLORE_GAP_ROUNDS = 2         # 2026-10-26 explore reliability: rounds of rays aimed at the sides not met yet before explore
                               # gives up (was 1). Chosen: each round costs 2-4 min. DOE note: yes, simulator: 1 / 2 / 3
                               # rounds, pass rate against minutes, over the batch's seeds
EXPLORE_MIN_ROBOTS = 2         # 2026-10-26 (the 09:55 session analysis): explore's camera fit is used only when at least
                               # this many robots' paths pass their own check (a single robot's scale is unconfirmed).
                               # A one-robot session (--robots Right) keeps explore as before. No DOE: a rule
EXPLORE_CRAWL_CMS = 4.0        # Nick 2026-10-24: rays, probes and the square crawl (DOE-explore-crawl.md decides it)
EXPLORE_FAST_CMS = 10.0        # e02_cms otherwise (the firmware's default): returns over ground just driven
EXPLORE_PROBE_CM = 12.0        # chosen: the room check's short drive each way
EXPLORE_RAY_CLEAR_CM = 25.0    # Nick 2026-10-24: no ray passes nearer another robot than this
# ... and, since 2026-10-25 (15:10: Left's home leg pushed Center), no home or return leg either:
EXPLORE_LEG_SHORTEN = (0.75, 0.5, 0.25)   # chosen: a blocked leg is tried this much shorter. No DOE: any clear part
                                          # is better than none; how close it gets is EXPLORE_RAY_CLEAR_CM's job
# An object beyond an open edge is not a board (Nick, 2026-10-25, after 15:10: a light object on the floor past the
# bottom edge read as "board" twice). A ray the front stop ends is not taken as a board yet: the robot creeps on,
# its edge guard on and its front stop moved in to EXPLORE_CREEP_FRONT_CM, in EXPLORE_CREEP_STEP_CM steps. Its line
# sensors (or the bottom zone) finding the edge first: an OPEN edge, the object logged as beyond it. Reaching the
# object with floor still under it: a blocked side it can approach (flush with the plywood or not).
EXPLORE_CREEP_FRONT_CM = 3.0    # chosen: the front stop while creeping. Line sensors 3.5 cm and the front sensor 4.5 cm
                                # ahead of the axle, the bumper ~6 cm, the bottom zone ~9 cm: at 3 cm from the object the
                                # bumper is 1.5 cm clear, the line sensors have checked the floor to 4 cm short of it and
                                # the bottom zone looks past it. (2 cm in the first simulator runs: slanting approaches
                                # touched with the bumper's corner first, the wheels slipped, and the camera fit's wheel
                                # scales went 10 % off.) At 15:10 the object stood 5-10 cm past the edge, so the edge lay
                                # 0-3 cm ahead of the line sensors at the 7 cm stop: 4 cm of creep covers it.
                                # DOE note: yes, with the crawl DOE (DOE-explore-crawl.md): objects 0-10 cm past an open
                                # edge and boards 0-5 cm out, 2 / 3 / 4 cm, count right calls against bumper contacts
EXPLORE_STALL_SHARE = 0.4      # 2026-10-26 explore reliability: a creep step the camera saw move under this share of its
                                # wheels' count is against something (simulator: 0.3 of 2.9 cm, 4 steps in a row; a free
                                # step moves 0.9-1.1 of it). DOE note: yes, simulator first (stall share against true
                                # contact, 0.3 / 0.4 / 0.6), then the board: slanted creeps into a board at 30 / 45 / 60 deg
EXPLORE_BUMPER_AHEAD_CM = 6.0   # the bumper's front, ahead of the axle (the comment above: "the bumper ~6 cm"); where a
                                # stalled creep met the object. DOE note: no, measure it on a robot (a ruler)
EXPLORE_CREEP_STEP_CM = 3.0     # chosen: one creep leg; the camera sees where each ends before the next. DOE note: no, the
                                # robot's own edge guard is the stop; the step only paces the camera's checks
EXPLORE_CREEP_CMS = 2.0         # chosen: creep speed, half the ray's crawl ... and never more than
EXPLORE_CREEP_UNSEEN_CM = 1.0   # ... this far driven in one camera lag (speed <= 1 cm / lag: 2 cm/s needs lag <= 0.5 s;
                                # at 15:10's worst second, lag 0.59 s, 1.7 cm/s). DOE note: yes, the crawl DOE's speed
                                # factor (2 / 4 cm/s) at two camera rates sets both; the edge guard's stop distance is
                                # what it measures
EXPLORE_CREEP_MAX_CM = 15.0     # chosen: never creep further than this past the front stop (7 cm reading + margin).
                                # No DOE: a cap
FRONT_STOP_CM = 7.0             # the firmware's safe_front_cm (params.h), put back after a creep
EXPLORE_HOME_MOVE_CM = 15.0    # chosen: a home leg that would pass a robot is aimed up to this far to one side of its
                               # target (session-1510 report, question 3 to Nick). No DOE: a setup preference
MAKE_WAY_EDGE_CM = 20.0        # chosen: a robot moving aside stays this far from an open edge
EXPLORE_HOME_FRAC = 0.35       # chosen: explore ends this share of the way from its start towards its rays' middle
EXPLORE_HOME_IN_CM = 30.0      # chosen: at most this far in from its start
EXPLORE_EDGE_BACK_CM = 9.5    # chosen: clears most of the bottom zone's ~9 cm look-ahead; under the firmware's 10 cm
EXPLORE_BACK_ALLOW_CM = 1000   # e02_back_max_cm while exploring (only edge back-offs go backwards)
BACK_ALLOW_CM = 10             # the firmware's default, put back after explore


class ExploreShort(Exception):
    """Explore's rays did not meet all four sides yet (more rays, then try again)."""


class ExploreFailed(Exception):
    """Explore's results did not pass their checks: the runner falls back to a calibration on hand."""


class Runner:
    # defaults for objects made without __init__ (the tests' stand-ins); __init__ sets each one for real
    held_out = frozenset()
    hold = threading.Event()
    hold_answer = None
    hold_owner = None
    last_hold_end = 0.0
    stop_requested = False
    hold_auto_refind = False
    clock = None              # the session timeouts (e02_timeout.SessionClock); None: none (the tests' stand-ins)
    end_reason = None         # dict(key, why, ...) once the session's end is decided (e02_state.json "session_end")
    wrap_up = False           # the hard cap is reached: no new run starts
    timed_out = False         # a timeout has stopped the session
    seek = set()
    leg_live = {}
    leg_kind = {}
    leg_val = {}
    leg_end = {}
    speed_now = {}
    seek_end = {}
    history_raw = None
    guard = None
    degrade = DG.Degrade("")
    trust = TR.SensorTrust()
    trust_path = os.path.join(REPO, "logs", "sensor_trust-test.json")

    def __init__(self, a, state, session_dir):
        self.a = a
        self.st = state
        self.dir = session_dir
        self.logf = open(os.path.join(session_dir, "e02_log.txt"), "a", encoding="utf-8")
        self.t_launch = time.time()
        # session timeouts (Nick, 2026-10-26: a hold sat unanswered all night): e02_timeout.py
        self.clock = TO.from_args(a, self.t_launch)
        self.end_reason = None
        self.end_lock = threading.Lock()
        self.ended = threading.Event()    # set when launch() reaches its own end (its finally)
        self.wrap_up = False
        self.timed_out = False
        self.phase = "setup"
        self.order = ""
        self.active = None
        self.trackf = open(os.path.join(session_dir, "track.csv"), "a", newline="")
        self.trackw = csv.writer(self.trackf)
        if os.path.getsize(os.path.join(session_dir, "track.csv")) == 0:
            self.trackw.writerow(["t", "robot", "x_cm", "y_cm", "order", "phase", "source", "lock_score"])
        self.heads = {}               # robot -> [heading deg or None, measured bool]
        self.unavailable = set()
        self.exploring = False        # explore: no calibration and no board known yet
        self.skip_conds = set()       # conditions put aside for this session (backup: camera-dependent ones failing)
        self.in_runs = False          # the session's runs have started (backups apply only then)
        self.last_result = ""
        self.kc_stop = threading.Event()
        # the camera's error by place (e02_camerr.py; 2026-10-24 review): tape marks and finds; KC fixes are
        # corrected with it and weighted by it
        self.camerr = CE.CameraErrorMap(state.get("camera_error_samples"))
        # how much each sensor is trusted, learned (e02_trust.py; Nick 2026-10-24), and the --degrade test
        self.trust_path = trust_path(a)
        base = trust_base_path(a)
        if base and not state.get("trust_session_started") and not getattr(a, "trust_file", None):
            self.trust = TR.load(base)            # a degraded test: from the trained trust, kept apart from it
        else:
            self.trust = TR.load(self.trust_path)
        if not state.get("trust_session_started"):
            self.trust.new_session()
            state["trust_session_started"] = round(time.time(), 1)
        self.degrade = DG.Degrade(getattr(a, "degrade", "") or "", seed=int(time.time()) % 100000)
        # the runner's own speed against its baseline, and a planned load (e02_load.py; Nick 2026-10-25)
        self.speed = LD.SpeedWatch()
        self.load = None
        if getattr(a, "load", None):
            self.load = LD.Load(a.load, getattr(a, "load_on_s", 300.0), getattr(a, "load_off_s", 300.0),
                                log=self.say, mark=self.mark,
                                record=lambda r_: self.st.setdefault("load_blocks", []).append(r_))
        self.quality = CAM.QualityMap()      # how well the camera sees a robot, place by place (2026-10-24)
        self.marks_unconfirmed = 0
        self.watchdog_fired = False
        self.watch_stop = None
        self.watch_quit = threading.Event()
        self.hold = threading.Event()     # set while the session waits for Nick (hold_for_nick)
        self.hold_lock = threading.Lock()
        self.hold_answer = None
        self.hold_owner = None
        self.stop_requested = False       # Nick answered s at a hold: every move from now on ends the session
        self.last_hold_end = 0.0
        self.held_out = set()             # robots Nick left out at a hold: they do not move again this session
        self.hold_auto_refind = False     # Nick's answer "a": lost robots are found again without asking (border still asks)
        self.guard = None                 # (camera, board) for the border while exploring (no board of its own yet)
        self.seek = set()                 # robots on a forward crawl looking for the edge on purpose (border exempt)
        self.seek_end = {}                # robot -> when its last edge-seeking crawl ended (the border ignores older pictures)
        self.leg_live = {}                # robot -> time its current leg was sent (cleared when it ends)
        self.leg_kind = {}                # robot -> "go" / "turn": its current (or last) leg
        self.leg_val = {}                 # robot -> its current leg's value (cm or deg)
        self.leg_end = {}                 # robot -> when its last leg ended
        # 2026-10-25 15:45 fixes
        self.turnwatch = TW.TurnWatch()   # fix 1: turns that keep failing (e02_turnwatch.py)
        self.turn_done = {}               # robot -> its own heading change over its last turn leg (deg, None unknown)
        self.turn_start_xy = {}           # robot -> the camera's position when its current turn began (fix 5: spins)
        self.turned_since_measure = {}    # robot -> one of its own turns since its heading was last measured
        self.edge_stop_t = {}             # robot -> when its own edge guard last stopped it (fix 4: back-outs)
        self.go_end = {}                  # robot -> when its last drive ended (fix 6: no fix from a held spot)
        self.src_log = []                 # [(t, tracker source)] of the latest pictures (fix 6)
        self.turn_speed_now = {}          # robot -> the turn speed (rpm) last set on it (fix 5)
        self.help_waiting = None          # the robot whose help call waits for Nick (the live page sounds)
        self.moves = {}               # robot -> dict(run ("-" = idle legs), why): it is taking legs now
        self.leg_ids = {}
        self.last_lid = {}            # robot -> id of the last leg sent
        self.main_lid = {}            # robot -> id of the last leg() asked for (not a refind's turns)
        self.watch_count = defaultdict(int)  # robot -> watchdog stops in the current drive
        self.disagree = defaultdict(int)  # robot -> camera / wheels disagreements in a row
        self.refinding = set()            # robots being found again (no camera/wheels check inside: no recursion)
        self.runner_stopped = set()       # robots the runner itself sent `stop` during a run (not the watchdog)
        self.fresh_id = {}            # robot -> True: its position came from identify() and it has not driven since
        self.watch_edge_cm = None     # the watchdog's open-edge distance (None: PL.WATCH_EDGE_CM; mapping: 0)
        self.bricks_view = None       # the brick demo's drawing on the live view (bricks, path, tracks), or None
        self.bricks_guard = None      # the brick demo's free_space while objects are on the board (camera watchdog)
        self.link = self.listener = self.watch = self.pi = None
        self.feed = self.tracker = self.history = self.cam = self.board = None
        self.front_seen = {}                 # "all" -> [front reading - map distance]
        # each robot's front-sensor offset and tolerance, learned every session from earlier ones (e02_front.py)
        self.front = FR.FrontModel(os.path.join(REPO, "logs", "front_learned%s.json" % (
            "-dryrun" if getattr(a, "dry_run", False) else "")))
        self.front_bad = {}                  # robot -> words: the last leg's front reading disagreed with the map
        self.fix_t = {}                      # robot -> when the camera last told it where it is
        self.speed_now = {}                   # e02_cms last set per robot (explore crawls)
        self.return_after = []                # robots that stepped aside, to go back to their spots
        self.unc = {}                         # item 4: driven and turned since the last fix, per robot
        # item 4's model, learned every session from the robot's own camera corrections (e02_unc.py)
        self.uncm = UNC.UncModel(os.path.join(REPO, "logs", "unc_learned%s.json" % (
            "-dryrun" if getattr(a, "dry_run", False) else "")))
        self.lost_place = set()              # robots that moved unseen with no trusted pose since (a failed own find)

    # --- talking ----------------------------------------------------------------------------------
    def say(self, msg, quiet=False):
        line = "[%s] %s" % (dt.datetime.now().strftime("%H:%M:%S"), msg)
        if not quiet:
            print(line, flush=True)
            if getattr(self, "view", None) is not None:
                self.view.log(line)
        self.logf.write(line + "\n")
        self.logf.flush()

    def planned_schedule(self):
        """The schedule this session will run (the same one before setup, to show, and when it starts)."""
        a = self.a
        nb = a.blocks or PL.blocks_for_time(a.hours, a.run_s, a.find_s, PL.SETUP_S, len(self.st["robots"]))
        sched = PL.make_schedule(self.st["robots"], nb, self.st["seed"])
        if a.routes != "1,2,3":
            keep = {int(v) for v in a.routes.split(",")}
            sched = [r for r in sched if r["route"] in keep]
            for i, r in enumerate(sched):
                r["order"] = i + 1
        return sched

    def recorder_ok(self):
        """The Pi recorder at a block start (or after 3 unconfirmed marks): the overhead stream and the notes link
        must answer. If not: pause, tell Nick, check again every 30 s until both answer (no runs without the
        video that measures them)."""
        a = self.a
        if not TKW.recorder_used(a):
            return True                         # production: no recorder, no video to protect
        t_pause = None
        while True:
            ok1, w1, _, _ = CAM.check_stream(a.pi, a.overhead_port)
            if not ok1 and self.feed is not None and self.feed.last_t is not None and \
                    time.time() - self.feed.last_t < 3.0:
                # the runner's own feed has fresh pictures: the camera is fine, only a second connection was slow
                # (2026-10-23 12:10: a 6 GB copy from the Pi filled the WiFi; the check paused the session)
                ok1 = True
            # --no-recording: the Pi runs camrec.py --no-record (live picture only: no video, no notes)
            notes = True if getattr(a, "no_recording", False) else CAM.notes_answer(a.pi)
            if ok1 and notes:
                self.marks_unconfirmed = 0
                return True
            self.say("PAUSED: the Pi recorder is not answering (overhead camera: %s; notes: %s). Check the Pi's SSH "
                     "window: if camrec.py stopped, start it again with the same command as at the start. This "
                     "checks again every 30 s and carries on by itself." % ("OK" if ok1 else w1,
                                                                         "OK" if notes else "no answer"))
            self.st.setdefault("recorder_pauses", []).append([round(time.time(), 1), ok1, notes])
            self.st.save()
            print("\a", end="", flush=True)
            # the session timeouts (2026-10-26): this pause waits on Nick like a hold, so the same limits end it
            t_pause = t_pause or time.time()
            t_sess, key = self.session_deadline(t_pause, None)
            if t_sess is not None:
                self.say("  (%s.)" % self.end_page_words(t_sess))
            for _ in range(60):
                if self.stop_requested:
                    self.raise_if_stopped()
                    return False
                if self.wrap_up or (t_sess is not None and time.time() >= t_sess):
                    self.session_timeout(TO.CAP if self.wrap_up else key, t_pause, "the recorder check's pause")
                    self.raise_if_stopped()
                    return False
                time.sleep(0.5)

    SNAP_MIN_S = 1.0          # at most one snapshot a second (marks come in bursts)

    def snapshot(self, text):
        """A still of the live view (the picture with the runner's drawings) at an important moment, into
        <session>/snapshots/ (Nick, 2026-10-23: a session run without video still gets its important frames; and
        the summary's screenshots). Never stops the session."""
        try:
            import cv2
            now = time.time()
            if now - getattr(self, "snap_t", 0.0) < self.SNAP_MIN_S:
                return
            view = getattr(self, "view", None)
            img = view.frame_bgr() if view is not None else (self.feed.last_img if self.feed is not None else None)
            if img is None:
                return
            self.snap_t = now
            d = os.path.join(self.dir, "snapshots")
            os.makedirs(d, exist_ok=True)
            slug = re.sub(r"[^A-Za-z0-9]+", "-", text)[:60].strip("-")
            name = time.strftime("%H%M%S", time.localtime(now)) + "_" + slug + ".jpg"
            cv2.imwrite(os.path.join(d, name), img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        except Exception:
            pass

    FRAMES_KEEP_S = 150.0      # pictures kept for the measured frames (a run and its find; about 60 MB as JPEG)

    def measured_frames(self, r, items):
        """Nick, 2026-10-23: the pictures behind each run's numbers. For each (name, laptop time, measured point,
        reference point): the kept picture nearest that time (0.05 s after it, as the measurement starts there), with
        the measured point (green cross) and the reference (white ring: the intended far point, or the start for
        the end) drawn, into snapshots/run<order>_<robot>_<cond>_<route>_<name>.jpg. Never stops the session."""
        try:
            import cv2
            if self.feed is None or not getattr(self.feed, "keep_s", 0) or self.cam is None:
                return
            d = os.path.join(self.dir, "snapshots")
            os.makedirs(d, exist_ok=True)
            for name, t, pt, ref in items:
                if t is None:
                    continue
                img = self.feed.frame_at(t + 0.05)
                if img is None:
                    continue
                v = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img.ndim == 2 else img
                for q, col, kind in ((ref, (255, 255, 255), "ring"), (pt, (0, 255, 0), "cross")):
                    if q is None:
                        continue
                    u, w = self.cam.project((q[0], q[1], CAM.ROBOT_TOP_CM))
                    if not (np.isfinite(u) and np.isfinite(w)):
                        continue
                    if kind == "ring":
                        cv2.circle(v, (int(u), int(w)), 10, col, 1)
                    else:
                        cv2.drawMarker(v, (int(u), int(w)), col, cv2.MARKER_CROSS, 14, 2)
                cv2.putText(v, "run %s %s %s %s: %s" % (r.get("order"), r.get("robot"), r.get("cond"),
                                                         PL.ROUTE_SHORT.get(r.get("route"), r.get("route")), name),
                            (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
                fn = "run%02d_%s_%s_%s_%s.jpg" % (int(r.get("order") or 0), r.get("robot"), r.get("cond"),
                                                  PL.ROUTE_SHORT.get(r.get("route"), r.get("route")), name)
                cv2.imwrite(os.path.join(d, re.sub(r"[^A-Za-z0-9_.-]+", "-", fn)), v,
                            [cv2.IMWRITE_JPEG_QUALITY, 90])
        except Exception as e:
            self.say("  (measured frames not saved: %s)" % e, quiet=True)

    def mark(self, text):
        """A note in both camera videos (camrec.py writes it to each camera's events.csv), and a snapshot."""
        self.snapshot(text)
        try:
            TKW.on_mark(self, text)            # a hold, a stop, a help call, a failed turn: a still and a clip (frames/)
        except Exception:
            pass
        ok = self.pi.send(text) if self.pi else False
        self.marks_unconfirmed = 0 if ok else self.marks_unconfirmed + 1
        self.say("  [video mark] %s%s" % (text, "" if ok else "  (the Pi didn't confirm it)"), quiet=ok)
        self.st.setdefault("notes", []).append([round(time.time(), 3), text, bool(ok)])

    RESCUE = "\x00rescue"         # what a robot's hold gets back when nobody answered by its deadline (item 7)

    def web_ask(self, prompt, default, timeout_s, hold=False, image=None, t_asked=None, rescue_ok=False):
        """The Pi run (--web-ask): the question goes on the live page (any phone or computer on the WiFi answers
        it), the console can answer too; after timeout_s with no answer, the default (said loudly). A question
        from anywhere else waits while a hold's question is on the page (the page shows one question: 2026-10-24
        review, a recovery's question replaced the hold's and the hold could never be answered). image: a picture
        the question is about (a file path); the page shows it inside the question (Nick, 2026-10-25: he answered
        the outline question blind).
        Session timeouts (2026-10-26, e02_timeout.py): if nobody answers by the session's deadline for this wait
        (t_asked: when it was first put to Nick, e.g. a hold's start), the session ends (session_timeout) and this
        returns None; the page shows "No answer by HH:MM ends the session and uploads" under it."""
        v = getattr(self, "view", None)
        if v is None:
            return None
        while not hold and self.hold.is_set() and self.hold_owner is not threading.current_thread():
            time.sleep(0.3)
        if getattr(self, "stop_requested", False):  # a hold ended the session while this question waited for it
            self.raise_if_stopped()
            return None
        t0 = time.time() if t_asked is None else t_asked
        t_sess, key = Runner.session_deadline(self, t0, timeout_s)
        ends = Runner.end_page_words(self, t_sess)
        self.say("QUESTION (answer on the live page): " + prompt.strip())
        if ends:
            self.say("  (%s.)" % ends)
        box = []
        if sys.stdin is not None and sys.stdin.isatty():
            threading.Thread(target=lambda: box.append(self._input("")), daemon=True).start()
        t_end = None if timeout_s is None else time.time() + timeout_s
        timed = False
        with v.lock:
            qid = int(time.time() * 1000)
            v.question = dict(id=qid, prompt=prompt.strip(), default=default, timeout_s=timeout_s,
                              ends_at=None if t_sess is None else round(t_sess, 1), ends_words=ends)
            if image and os.path.exists(image):
                v.question["image"] = os.path.abspath(image)
        try:
            while True:
                now = time.time()
                if t_sess is not None and now >= t_sess:
                    ans, timed = None, True
                    break
                if t_end is not None and now >= t_end:
                    ans = None
                    break
                if getattr(self, "timed_out", False):                # another wait's timeout (or the hard cap) ended the session
                    ans = None
                    break
                with v.lock:
                    if qid in v.answers:
                        ans = v.answers.pop(qid).strip()
                        break
                if box:
                    ans = box[0]
                    break
                time.sleep(0.3)
        finally:
            with v.lock:
                if v.question and v.question["id"] == qid:
                    v.question = None
        if timed:
            if rescue_ok and key != TO.CAP:
                return Runner.RESCUE                  # item 7: the hold rescues its robot instead of ending the session
            Runner.session_timeout(self, key, t0, prompt)
            return None
        if ans is None and getattr(self, "timed_out", False):
            return None
        if ans is None:
            self.say("  (no answer in %d min: the default, %r, is taken)" % (round((timeout_s or 0) / 60), default))
            self.mark("e02 question timed out: %s" % prompt.strip()[:60])
        else:
            self.say("  answer: %r" % ans)
        return ans

    def console_wait(self, prompt, t_asked=None, rescue_ok=False):
        """The console's answer (no live page), with the session timeouts: the answer, None at end of input (no
        console), or None once the session's deadline for this wait passed (the session is then ending); with
        rescue_ok, Runner.RESCUE instead (a robot's hold: item 7), unless it is the hard cap."""
        t0 = time.time() if t_asked is None else t_asked
        t_sess, key = Runner.session_deadline(self, t0, None)
        if t_sess is None:
            try:
                return input(prompt).strip()
            except EOFError:
                return None
        self.say("  (%s.)" % Runner.end_page_words(self, t_sess))
        box = []

        def read():
            try:
                box.append(input(prompt).strip())
            except EOFError:
                box.append(None)
        threading.Thread(target=read, daemon=True).start()
        while not box and time.time() < t_sess:
            time.sleep(0.3)
        if box:
            return box[0]
        if rescue_ok and key != TO.CAP:
            return Runner.RESCUE
        Runner.session_timeout(self, key, t0, prompt)
        return None

    def ask(self, prompt, default="y", image=None):
        """image: the picture the question is about (shown inside the question on the live page)."""
        if self.a.auto:
            self.say(prompt + " -> " + default + " (automatic)")
            return default
        if getattr(self.a, "web_ask", False):
            ans = self.web_ask(prompt, default, WEB_ASK_TIMEOUT_S, image=image)
            if getattr(self, "timed_out", False):
                Runner.raise_if_stopped(self)    # (the main thread: the session ends here)
            return default if ans is None else (ans or default)
        ans = self.console_wait(prompt)
        if getattr(self, "timed_out", False):
            Runner.raise_if_stopped(self)
        return default if ans is None else ans

    def ask_timeout(self, prompt, timeout_s, image=None):
        """input() that gives up after timeout_s (None): an unattended session must never wait for ever."""
        if self.a.auto:
            return None
        if getattr(self.a, "web_ask", False):
            ans = self.web_ask(prompt, "", timeout_s, image=image)
            if getattr(self, "timed_out", False):
                Runner.raise_if_stopped(self)
            return ans
        box = []
        th = threading.Thread(target=lambda: box.append(self._input(prompt)), daemon=True)
        th.start()
        if timeout_s is None:
            t_sess, key = Runner.session_deadline(self, time.time(), None)
            th.join(None if t_sess is None else max(0.0, t_sess - time.time()))
            if not box and t_sess is not None:
                Runner.session_timeout(self, key, None, prompt)
                self.raise_if_stopped()
        else:
            th.join(timeout_s)
        return box[0] if box else None

    # --- session timeouts (Nick, 2026-10-26; e02_timeout.py) ------------------------------------------------
    def session_deadline(self, t_asked, own_timeout_s=None):
        """(t, key): when a wait on Nick begun at t_asked ends the session, or (None, None)."""
        c = getattr(self, "clock", None)
        if c is None:
            return None, None
        return c.wait_deadline(t_asked, own_timeout_s)

    def end_page_words(self, t_end):
        """'No answer by 23:05 ends the session and uploads' ('' for no deadline). Uploads: the Pi run."""
        c = getattr(self, "clock", None)
        if c is None or t_end is None:
            return ""
        return c.page_words(t_end, uploads=bool(getattr(self.a, "on_pi", False)))

    def note_end(self, key, why, **extra):
        """The reason the session ended (the first one decided wins): e02_state.json "session_end", the log."""
        lock = getattr(self, "end_lock", None) or threading.Lock()
        with lock:
            if self.end_reason is not None:
                return False
            self.end_reason = dict(key=key, why=why, t=round(time.time(), 1), **extra)
        try:
            self.st["session_end"] = dict(self.end_reason)
            self.st.save()
        except Exception:
            pass
        return True

    def session_timeout(self, key, t_asked=None, what=""):
        """Nobody answered in time, or the hard cap: every robot stopped, the state saved, and the session ends the
        way an "s" at a hold does (the main thread stops at its next move or question; ensure_end makes sure)."""
        c = getattr(self, "clock", None)
        why = c.why(key, t_asked) if c is not None else str(key)
        self.note_end(key, why, asked=None if t_asked is None else round(t_asked, 1),
                      waiting_on=(what or "").strip()[:160])
        if getattr(self, "timed_out", False):
            return
        self.timed_out = True
        self.say("!" * 78)
        self.say("SESSION ENDS (%s): every robot is stopped, the state is saved, and the runner finishes so the "
                 "upload runs and the recorder stops. --resume carries the rest on later." % why)
        self.say("!" * 78)
        try:
            self.mark("e02 session timeout %s" % key)
            self.alert("timeout", None, why)
        except Exception:
            pass
        try:
            self.stop_everyone()
        except Exception:
            pass
        self.stop_requested = True
        try:
            self.st.save()
        except Exception:
            pass
        if getattr(self, "ended", None) is not None:
            threading.Thread(target=self.ensure_end, daemon=True).start()

    def interrupt_main(self):
        """Like Ctrl+C in the runner's window (also breaks a sleep or a console input())."""
        os.kill(os.getpid(), signal.SIGINT)

    def ensure_end(self, wait_s=None):
        """After a timeout: if the runner has not reached its own end in ENSURE_END_S (stuck where nothing checks,
        e.g. the recorder check's wait), interrupt it like Ctrl+C. Never after its end has begun."""
        if self.ended.wait(TO.ENSURE_END_S if wait_s is None else wait_s):
            return
        self.say("  (the runner had not finished %d s after the timeout: interrupting it like Ctrl+C)" % (
            TO.ENSURE_END_S if wait_s is None else wait_s))
        self.interrupt_main()

    def cap_watch(self, grace_s=None):
        """The hard cap (planned hours + --hard-cap-min): no new run starts (session() checks wrap_up), the current
        one finishes and parks if it does so within WRAP_GRACE_S; otherwise every robot is just stopped
        (session_timeout). A hold or question open at the cap ends the session at once (its deadline is the cap)."""
        c = getattr(self, "clock", None)
        if c is None or c.cap is None:
            return
        while not self.ended.is_set() and time.time() < c.cap:
            self.ended.wait(min(30.0, max(0.05, c.cap - time.time())))
        if self.ended.is_set():
            return
        self.wrap_up = True
        g = TO.WRAP_GRACE_S if grace_s is None else grace_s
        self.say("HARD CAP (%s): no new run starts; the current one finishes and parks if it can within %d s, else "
                 "every robot is stopped. Then the session ends and uploads." % (c.why(TO.CAP), g))
        try:
            self.mark("e02 hard cap")
        except Exception:
            pass
        if self.ended.wait(g):
            return
        self.session_timeout(TO.CAP, None, "the hard cap's wrap-up (phase %s)" % getattr(self, "phase", "?"))

    @staticmethod
    def _input(prompt):
        try:
            return input(prompt).strip()
        except EOFError:
            return ""

    def send(self, robot, cmd):
        if cmd.startswith(HOLD_MOVE_CMDS):
            if self.hold.is_set() and threading.current_thread() is threading.main_thread() \
                    and self.hold_owner is not threading.current_thread():
                # the watchdog is holding the session for Nick: nothing moves until he answers
                while self.hold.is_set():
                    time.sleep(0.2)
            self.raise_if_stopped()
            if robot in self.held_out:
                self.say("    (%s was left out at a hold: not sent %s)" % (robot, cmd), quiet=True)
                return False
            if threading.current_thread() is threading.main_thread():
                lg = getattr(self, "link_gate", None)            # the link is too bad: moves wait (item 1e)
                if lg is not None:
                    lg(robot, cmd)
                    self.raise_if_stopped()
            sgw = getattr(self, "speed_guard_wait", None)       # (absent on the tests' stand-in runners)
            if sgw is not None and threading.current_thread() is threading.main_thread():
                sgw(robot, cmd)
            ws = getattr(self.link, "wait_settings", None)
            if ws is not None:
                # item 1b: every setting sent to this robot is confirmed (read back) before it moves
                left = ws(robot)
                if left:
                    self.say("!!! LINK: %s's settings %s were not confirmed in %.0f s: the move goes anyway (they are "
                             "sent again)." % (robot, ", ".join(left), LNK_SET_WAIT_S))
                    self.st.setdefault("settings_unconfirmed", []).append([round(time.time(), 1), robot, left, cmd])
                told = self.__dict__.setdefault("settings_failed_told", set())
                for r_, k_, v_ in self.link.failed_settings(robot):
                    if (r_, k_, v_) not in told:
                        told.add((r_, k_, v_))
                        self.say("!!! LINK: %s never confirmed %s = %g after %d tries: it may still have its old "
                                 "value." % (r_, k_, v_, LNK.SET_TRIES))
                        self.st.setdefault("settings_failed", []).append([round(time.time(), 1), r_, k_, v_])
                        self.alert("link", r_, "%s never confirmed %s = %g" % (r_, k_, v_))
        self.link.send_name(robot, cmd)
        self.say("    -> %s: %s" % (robot, cmd), quiet=True)
        return True

    # --- camera -----------------------------------------------------------------------------------
    def on_sample(self, s):
        if getattr(self, "lab", None) is not None and self.tracker is not None and self.tracker.active:
            try:
                self.lab.on_camera(self.tracker.active, s[0], s[1], s[2])
            except Exception:
                pass
        if self.tracker is not None and self.tracker.active and self.feed is not None:
            try:
                u, v = self.tracker._px(s[1], s[2])
                self.quality.add(self.feed.last_img, u, v, s[1], s[2], getattr(self.tracker, "source", ""), s[0])
            except Exception:              # a display-grade measurement: never break the camera loop
                pass
        if self.tracker is not None and self.tracker.active:
            sl = self.__dict__.setdefault("src_log", [])
            sl.append((s[0], getattr(self.tracker, "source", "") or ""))      # fix 6: was it really seen?
            if len(sl) > 400:
                del sl[:200]
            lk = getattr(self.tracker, "lk", None)
            self.trackw.writerow(["%.3f" % s[0], self.tracker.active, "%.2f" % s[1], "%.2f" % s[2], self.order, self.phase,
                                  getattr(self.tracker, "source", "") or "",
                                  "%.2f" % lk.score if lk is not None and lk.locked else ""])

    PERF_COLS = ["t", "fps_handled", "dropped", "ms_mean", "ms_max", "age_mean_s", "age_max_s", "wait_s", "source",
                 "phase"]

    def on_perf(self, row):
        """perf.csv, one row a second: is the laptop keeping up with the camera (Nick, 2026-10-23)."""
        if getattr(self, "perfw", None) is None:
            path = os.path.join(self.dir, "perf.csv")
            new = not os.path.exists(path) or os.path.getsize(path) == 0
            self.perff = open(path, "a", newline="")
            self.perfw = csv.DictWriter(self.perff, fieldnames=self.PERF_COLS)
            if new:
                self.perfw.writeheader()
            self.perf_rows = []
        row = dict(row, phase=self.phase)
        self.perfw.writerow(row)
        self.perff.flush()
        ld_ = getattr(self, "load", None)
        lon = bool(ld_ is not None and ld_.on)
        sw = getattr(self, "speed", None)
        if sw is not None:
            ch = sw.feed(row, load_on=lon)
            if sw.base_fps is not None and not getattr(self, "speed_base_said", False):
                self.speed_base_said = True
                self.st["speed_baseline"] = dict(fps=round(sw.base_fps, 1), age_s=round(sw.base_age, 3))
                self.say("  Camera speed baseline: %.1f pictures a second handled, picture age %.2f s (slow below "
                         "%.1f or above %.2f s)." % (sw.base_fps, sw.base_age, sw.fps_frac * sw.base_fps,
                                                     sw.age_frac * sw.base_age))
            if ch == "slow":
                self.say("!!! SLOW: the runner handles %.1f pictures a second (baseline %.1f), picture age %.2f s "
                         "(baseline %.2f)%s." % (row["fps_handled"], sw.base_fps, row["age_mean_s"], sw.base_age,
                                                 " while the planned load is on" if lon else ""))
                self.mark("e02 slow")
            elif ch == "normal":
                sp = sw.spans[-1]
                self.say("  Speed back to normal (slow for %.0f s; worst %.1f pictures a second, age %.2f s)." % (
                    sp["t1"] - sp["t0"], sp["fps_min"], sp["age_max"]))
                self.mark("e02 speed normal")
            if ch:
                self.st["slow_spans"] = [dict(v) for v in sw.spans]
        row = dict(row, load=lon, slow=bool(sw is not None and sw.slow))
        self.perf_rows.append(row)
        if row["age_max_s"] > 1.0 and time.time() - getattr(self, "perf_warned", 0) > 60:
            self.perf_warned = time.time()
            self.say("  (camera: a picture was %.1f s old when its position was ready; %d dropped in the last second)"
                     % (row["age_max_s"], row["dropped"]), quiet=True)

    def perf_summary(self):
        rows = getattr(self, "perf_rows", None)
        if not rows:
            return None
        import statistics as STS
        q = lambda a, f: sorted(a)[min(len(a) - 1, int(f * len(a)))]
        ages = [r["age_mean_s"] for r in rows]
        return ("Camera processing: %.1f pictures a second handled (of ~25), %.0f ms a picture (95%%: %.0f ms), "
                "a picture's age when its position was ready: median %.2f s, 95%% %.2f s, worst %.2f s" % (
                    STS.median([r["fps_handled"] for r in rows]), STS.median([r["ms_mean"] for r in rows]),
                    q([r["ms_max"] for r in rows], 0.95), STS.median(ages), q([r["age_max_s"] for r in rows], 0.95),
                    max(r["age_max_s"] for r in rows)))

    def pos(self, robot=None, fresh_s=1.5):
        p = self.tracker.where()
        if getattr(getattr(self, "a", None), "fusion_steer", False) and (robot or self.tracker.active):
            # --fusion-steer (OFF by default, not yet tried on the board): the combined estimate where it was better
            r_ = robot or self.tracker.active
            return TKW.steer_pos(self, r_, p if (p is not None and self.tracker.active == r_) else None, fresh_s)
        if p is None or (robot and self.tracker.active != robot):
            return None
        if time.time() - p[0] > fresh_s:
            return None
        return p

    def settle(self, t_from=None, min_s=0.4, timeout=3.0):
        """The active robot's position once it stands still (median of min_s), or its latest one if the camera saw
        it in the last STALE_S. None if the camera has not seen it lately (the caller finds it again)."""
        self.camera_alive()
        end = time.time() + timeout
        while time.time() < end:
            sp = CAM.still_position(self.history.since(max(t_from or 0, time.time() - 2.0)), min_s=min_s)
            if sp:
                return sp
            time.sleep(0.1)
        p = self.tracker.where()
        if p is None or self.tracker.last_seen < time.time() - STALE_S:
            return None
        return (p[1], p[2])

    def camera_alive(self):
        """The camera feed thread must be running: if it died (an error in a frame), start a new one (review
        2026-10-22: a dead feed froze every position without anyone noticing)."""
        f = self.feed
        if f is None or self.a is None or getattr(self.a, "dry_run_feed_ok", False):
            return True
        if TKW.is_proc(f):
            if not f.is_alive():
                TKW.restart_feed(self)          # the tracker process stopped: a new one, set up the same way
            return True
        if f.is_alive():
            return True
        self.say("  (the camera feed stopped: %s; starting it again)" % (f.err or "no reason given"))
        nf = CAM.Feed(stream_url(self.a), self.tracker, self.history,
                      log=lambda m: self.say("  (camera: %s)" % m))
        nf.on_sample = f.on_sample
        nf.sample_filter = f.sample_filter
        nf.history_raw = f.history_raw
        nf.hooks = f.hooks
        nf.start()
        self.feed = nf
        nf.wait_frames(3, timeout=8)
        return False

    # --- runs and camera-steered moves -------------------------------------------------------------
    def start_run(self, robot, cmd="run e02", retry=True):
        """Send a `run ...` and wait for its RUNINFO. A robot that says it is not calibrated (switched off and
        on) gets its calibration first, then one more try. Returns the run id or None."""
        before = self.watch.started.get(robot)
        t0 = time.time()
        if cmd != "run find" and robot in getattr(self, "unc", {}):
            self.unc[robot]["clean"] = False     # its own driving is not in unc: no uncertainty correction until a fix
        if self.tracker is not None and self.tracker.active == robot:
            self.tracker.clear_expect()          # a run drives it: the camera must follow it anywhere
        self.send(robot, cmd)
        run = self.watch.wait_start(robot, before, 8)
        if run:
            return run
        said = self.listener.said(robot, t0, ("not calibrated", "busy", "unknown", "run what", "refuse"))
        if retry and any("not calibrated" in m.lower() for m in said):
            self.say("  %s was switched off and on since its calibration: calibrating it first." % robot)
            self.calibrate([robot])
            return self.start_run(robot, cmd, retry=False)
        if retry and any("busy" in m.lower() for m in said):
            self.send(robot, "stop")
            time.sleep(1.5)
            return self.start_run(robot, cmd, retry=False)
        self.say("  %s did not start `%s`%s" % (robot, cmd, (": " + said[-1]) if said else ""))
        return None

    def calibrate(self, robots, tries=0):
        """The compass check, checked (Nick, 2026-10-23 evening: all three failed it with a wheel over an open edge
        after explore, and the session carried on). A robot that fails it cannot be trusted to turn: the runner
        stops and asks (put its wheels on the board, then Enter to try again; `skip` to go on without it; `stop`)."""
        self._calibrate(robots)
        failed = [r for r in robots if r not in self.unavailable and not self.compass_ok(r)]
        if not failed:
            return
        self.say("  The compass check FAILED on %s. That usually means a wheel is off the board or it cannot "
                 "spin freely (pressed against a board, or a robot)." % ", ".join(failed))
        self.mark("e02 compass check failed %s" % ",".join(failed))
        if self.a.auto:
            if tries < 1:
                self.say("  (automatic mode: trying once more)")
                return self.calibrate(failed, tries + 1)
            for r in failed:
                self.unavailable.add(r)
            self.say("  (automatic mode: going on without %s)" % ", ".join(failed))
            return
        ans = self.ask("  %s: put %s wheels flat on the board, clear of the edges, then press Enter to check again "
                       "(or type skip to go on without %s, or stop to end): " % (
                           ", ".join(failed), "its" if len(failed) == 1 else "their",
                           "it" if len(failed) == 1 else "them"), "").lower()
        if ans.startswith("stop"):
            raise SystemExit("Stopped at the compass check (Nick).")
        if ans.startswith("skip"):
            for r in failed:
                self.unavailable.add(r)
            self.say("  Going on without %s." % ", ".join(failed))
            return
        return self.calibrate(failed, tries + 1)

    def compass_ok(self, robot):
        rows = [c for c in self.st.get("calibrations") or [] if c[1] == robot]
        return bool(rows) and str(rows[-1][3] or "").lower() in ("done", "ok")

    def _calibrate(self, robots, verify=True):
        """The compass check after power-on (`calibrate`, needed before find / e02 / gyro runs): spin and roll
        switched off, so it only wiggles in place on its park spot. All at once (each stays on its spot)."""
        robots = [r for r in robots if r not in self.unavailable]
        if not robots:
            return
        self.mark("e02 calibrate %s" % ",".join(robots))
        for r in robots:
            self.end_moves(r)
            for c in ["set calib_spin_deg 0", "set calib_roll_cm 0", "set calib_roll_min_cm 99"] + SAFETY_CMDS:
                self.send(r, c)
                time.sleep(0.05)
        runs = {}
        for r in robots:
            before = self.watch.started.get(r)
            self.send(r, "calibrate")
            runs[r] = (before, time.time())
        for r, (before, t0) in runs.items():
            run = self.watch.wait_start(r, before, 10)
            end = self.watch.wait_end(run, 90) if run else None
            self.say("  %s calibration (compass check): %s" % (r, end["result"] if end else "no answer"))
            self.st.setdefault("calibrations", []).append([round(t0, 1), r, run, end["result"] if end else None])
            if r in self.heads and self.heads[r][0] is not None:
                self.heads[r][1] = False             # the wiggle may have turned it a little
        self.st.save()
        if verify:
            for r in robots:
                self.compass_verify(r)

    COMPASS_CHECK_DEG = 15.0      # chosen: the check turn after a compass check; small enough to stay on its spot
    COMPASS_CHECK_TIMEOUT_MS = 2000   # chosen: a wrong sign spins the check turn at full speed: this caps it (about 90
    # deg at the real robots' 45 deg/s). DOE note: no, a cap
    COMPASS_TRIES = 3             # chosen: compass checks before a robot whose check turn keeps failing is left out (fix 1)

    def compass_verify(self, robot, tries=None):
        """Fix 1, root cause (2026-10-25 15:45): Right's compass check said "done" but learned the wrong turn sign:
        one 11.3 deg jump in its fused yaw at the start of the left wiggle made the yaw fall 4.6 deg while the
        gyro rate and the wheels both turned left (Left: +10.2, Center: +4.9). Every turn after it ran backwards
        to the timeout. Here a check turn of COMPASS_CHECK_DEG (its timeout capped) must turn the right way by its
        own heading; if not, the compass check runs again, up to COMPASS_TRIES in all, then the robot is left out
        (turn_fault_found). Returns True when it passed. Logged (compass_checks)."""
        tries = self.COMPASS_TRIES if tries is None else tries
        for k in range(tries):
            ok, why = self.compass_turn_check(robot)
            self.st.setdefault("compass_checks", []).append([round(time.time(), 1), robot, k + 1, ok, why])
            if ok:
                if k:
                    self.say("  %s: its compass check is good now (try %d)" % (robot, k + 1))
                return True
            self.say("!!! COMPASS: %s's check turn after its compass check failed (%s): checking the compass again "
                     "(%d of %d)" % (robot, why, k + 1, tries))
            self.mark("e02 compass check failed %s" % robot)
            if k < tries - 1:
                self._calibrate([robot], verify=False)
        self.__dict__.setdefault("compass_repairs", {})[robot] = True     # (no further compass repair)
        self.turn_fault_found(robot, "its compass check turn failed %d times (%s)" % (tries, why))
        return False

    def compass_turn_check(self, robot):
        """One check turn (+COMPASS_CHECK_DEG, then back) with the turn timeout capped: (ok, words). Judged by its
        own heading (LEG records): done, and turned the right way by at least half of it."""
        had = self.moves.get(robot)
        if had is None:
            self.moves[robot] = dict(run="-", why="compass check")
        try:
            self.link.send_name(robot, "set turn_timeout_ms %d" % self.COMPASS_CHECK_TIMEOUT_MS)
            time.sleep(0.05)
            ang = self.COMPASS_CHECK_DEG
            lid = self.next_id(robot)
            self.last_lid[robot] = lid
            state, info = self._send_leg_once(robot, "turn %.1f" % ang, 8.0, self.moves[robot], lid, False)
            st0 = self.listener.leg_start(robot, "-", lid)
            fin = self.listener.leg_final(robot, "-", lid)
            done = TW.turn_done_deg(num((st0 or {}).get("th_deg")), num((fin or {}).get("th_deg"))) \
                if (st0 and fin) else None
            if state in ("done", "dup") and done is not None and done * ang > 0 and abs(done) >= 0.5 * ang:
                self._send_leg_once(robot, "turn %.1f" % -ang, 8.0, self.moves[robot], self.next_id(robot), False)
                return True, "turned %.0f deg" % done
            if state == "refused":
                return False, "refused (%s)" % info
            return False, "%s%s" % (state + (" (%s)" % info if info else ""),
                                    "" if done is None else ", its own heading moved %.0f deg" % done)
        finally:
            self.link.send_name(robot, "set turn_timeout_ms %d" % (
                TURN_SLOW_TIMEOUT_MS if (getattr(self, "turn_speed_now", {}) or {}).get(robot) == TURN_SLOW_RPM
                else TURN_TIMEOUT_MS))
            if had is None:
                self.moves.pop(robot, None)

    FIND_HEAD_AGREE_DEG = 5.0     # chosen: the runner's heading goes with the fix before a find only within this of
    # the robot's own gyro heading (2026-10-24 analysis: Center's finds 13-24 cm off came from a camera heading set 2 s
    # before the find, up to 19 deg wrong; DOE: the find's heading correction against this limit)

    FIND_OWN_FRESH_S = 15.0       # chosen: its own heading counts against the runner's only if it reported it this
    # recently (a parked robot's gyro creeps unseen). DOE note: the creep of each robot standing still (the naming
    # check's gyro run) x the time robots stand parked between finds says how long an own heading stays good

    def find_heading_ok(self, robot):
        """(heading, position_only) for the fix before a find: the runner's heading, only if it was measured and the
        robot's own gyro heading agrees within FIND_HEAD_AGREE_DEG; otherwise position only (the robot keeps its own
        heading). A robot with no pose of its own gets the runner's heading as before (it needs one)."""
        h = self.heads.get(robot, [None, False])
        own = self.own_pose(robot)
        if own is None or own[2] is None:
            return h[0], False
        # its own heading as last reported (its last leg or find): while it stood still since, its gyro may have
        # crept unseen (2026-10-25 dry run: Right's uncorrected 0.57 deg/s drift took it 156 deg off while parked,
        # once fixes stopped writing the runner's heading over it); a measured runner heading then wins
        last = [r[0] for r in (self.listener.latest("LEG", robot), self.listener.latest("FIND", robot)) if r] \
            if getattr(self, "listener", None) is not None and hasattr(self.listener, "latest") else []
        stale = bool(last) and time.time() - max(last) > self.FIND_OWN_FRESH_S
        if h[0] is None or not h[1]:
            ok = False
        else:
            ok = stale or abs(ST.wrap180(h[0] - own[2])) <= self.FIND_HEAD_AGREE_DEG
        self.st.setdefault("find_heading_checks", []).append(
            [round(time.time(), 1), robot, None if h[0] is None else round(h[0], 1), bool(h[1]),
             round(own[2], 1), ok])
        if stale:
            self.st.setdefault("find_heading_stale", []).append([round(time.time(), 1), robot, round(max(last), 1)])
        return (h[0], False) if ok else (None, True)

    REAL_SOURCES = ("lock", "detector", "both")

    def seen_since_drive(self, robot, now=None):
        """Fix 6 (15:45: after a parking drive the tracker "held" the spot Left had left, and that spot, 15 cm off,
        went to Left as a camera fix). Within FIX_HELD_AFTER_GO_S of a drive, a fix needs a picture where the camera
        really saw the robot (lock, detector, both) after the drive ended: waits up to FIX_HELD_WAIT_S for one.
        Returns the time from which positions can be used (None: no drive lately, any picture will do), or False
        (only held spots: no fix; logged fix_skipped_held)."""
        now = time.time() if now is None else now
        t_go = (getattr(self, "go_end", None) or {}).get(robot)
        if t_go is None or now - t_go > FIX_HELD_AFTER_GO_S:
            return None
        end = now + FIX_HELD_WAIT_S
        while True:
            real = [t for t, src in list(getattr(self, "src_log", None) or []) if t > t_go + 0.2
                    and src in self.REAL_SOURCES]
            if real:
                return real[0]
            if time.time() >= end:
                break
            time.sleep(0.1)
        self.st.setdefault("fix_skipped_held", []).append([round(time.time(), 1), robot, round(time.time() - t_go, 1)])
        self.say("    %s: no camera fix: since its drive the camera only holds its old spot (not seen again)" % robot,
                 quiet=True)
        return False

    def fix_now(self, robot, heading=None, position_only=False):
        """Tell the robot where the camera sees it (`fix x y th t_wall`; while idle it replaces its pose).
        position_only: the heading is left to the robot's own gyro ("-").
        heading None: the runner's heading goes with the fix only if it was MEASURED (heads[robot][1]); otherwise
        "-" and the robot keeps its own gyro heading (2026-10-25 heading diagnosis, docs/build-reports/
        e02-oct25-heading.md fix 1: 100 of 112 30-deg+ heading errors were unmeasured runner headings written in by
        fixes). Only a robot with no pose of its own (the firmware refuses a first fix without a heading) still gets
        the unmeasured one; that fix is marked, and the uncertainty model does not learn from the stretch after it."""
        if self.tracker is None or self.tracker.active != robot:
            return None                  # the camera follows another robot (or none): no position for this one
        t_seen = self.seen_since_drive(robot)
        if t_seen is False:
            return None                  # fix 6: right after a drive, only a "held" spot: not sent
        p = self.settle(t_from=t_seen, min_s=0.3)
        if p is None:
            return None
        h0, meas = (self.heads.get(robot) or [None, False])[:2]
        own = self.own_pose(robot) if getattr(self, "listener", None) is not None else None
        guessed = False
        if position_only:
            h = None
        elif heading is not None:
            h = heading
            # an explicit heading counts as measured when it is the runner's measured one (callers pass measured
            # headings: a find, explore's frame, the bricks crossing, a heading push)
            guessed = not (meas and h0 is not None and abs(ST.wrap180(h - h0)) < 0.5)
        elif meas:
            h = h0
        elif own is None:
            h, guessed = h0, h0 is not None   # no pose at all: it needs some heading (begin_moves' placeholder)
        else:
            h = None                         # unmeasured: the robot's own gyro heading is the better one
        self.unc_learn(robot, own, p)        # how far off its own pose was (e02_unc), before the fix
        self.send(robot, "fix %.2f %.2f %s %.3f" % (p[0], p[1], "-" if h is None else "%.1f" % h, time.time()))
        self.fix_t[robot] = time.time()
        self.unc_reset(robot)
        if guessed:
            # the heading it now holds was not measured: a correction after this stretch says nothing about its
            # wheels and gyro (e02_unc would learn the placeholder's error); it is learned again after the next fix
            self.unc[robot]["clean"] = False
            self.st.setdefault("fix_unmeasured_heading", []).append([round(time.time(), 1), robot, round(h, 1)])
        time.sleep(0.15)
        return p

    HEADING_PUSH_DEG = 8.0        # heading fix 3 (e02-oct25-heading.md): chosen, about the noise of a 10 cm leg's
    # direction. DOE: doe-backlog.md "Heading push after a measured go leg" (logs, heading_legs.csv)

    def heading_push(self, robot, heading):
        """Heading fix 3: a go leg just MEASURED the heading (the camera saw it drive straight): if the robot's own
        heading differs by more than HEADING_PUSH_DEG, send it now (a fix with that heading), so a wrong heading the
        robot holds (a placeholder, a U run's end heading) is corrected on the first measuring leg. Returns True if
        it was sent. Logged (heading_pushes)."""
        if heading is None:
            return False
        own = self.own_pose(robot)
        if own is None or own[2] is None:
            return False
        d = ST.wrap180(heading - own[2])
        tsm = self.__dict__.setdefault("turned_since_measure", {})
        after_turn = tsm.pop(robot, False)
        tw = getattr(self, "turnwatch", None)
        if tw is not None:
            why = tw.correction(robot, d, after_turn)        # fix 1: its own heading far off after its own turns
            if why:
                self.turn_fault_found(robot, why)
                return False
        if abs(d) <= self.HEADING_PUSH_DEG:
            return False
        self.heads[robot] = [heading, True]
        if self.fix_now(robot, heading) is None:
            return False
        self.st.setdefault("heading_pushes", []).append([round(time.time(), 1), robot, round(own[2], 1),
                                                         round(heading, 1), round(d, 1)])
        self.say("    %s: its own heading was %.0f deg off the measured one: corrected" % (robot, d), quiet=True)
        return True

    def begin_moves(self, robot, why="moving"):
        """Camera-steered moves that are not measurements (parking, finding a robot, putting it back) are idle
        legs: `go` / `turn` with no run going (firmware 2026-10-21b: a board close in front no longer fails them,
        so the 2026-10-21a workaround of carrying them inside a camera-steered `run e02` is gone; idle legs are
        the safer of the two: each has its own travel cap of the leg + 10 cm, against the route + 140 cm + 30 cm
        of a run, and they leave no fake e02 runs in the logs). A camera fix first, so its own pose is sane."""
        if robot in self.moves:
            return True
        self.set_corrections(robot, self.move_corrections(robot))
        h, measured = (self.heads.get(robot) or [None, False])[:2]
        if measured and h is not None:
            self.fix_now(robot, h)
        elif self.own_pose(robot) is not None:
            # heading fix 2 (e02-oct25-heading.md): it has a pose of its own: keep its gyro heading (the +-90 deg
            # placeholder written here caused 76 of the 112 30-deg+ heading errors)
            self.fix_now(robot, position_only=True)
        else:
            # no pose at all: the firmware needs a heading with its first fix; the placeholder until the first
            # measuring leg (heading_push then corrects it)
            self.fix_now(robot, h if h is not None else self.board.face_xboard)
        self.moves[robot] = dict(run="-", why=why)
        return True

    def wait_mode1(self, robot, t0, timeout=8.0):
        """A camera-steered run takes legs only once it says so (after its settle pause and start flash)."""
        end = time.time() + timeout
        while time.time() < end:
            if self.listener.said(robot, t0, ("waiting for go",)):
                return True
            time.sleep(0.1)
        return False

    def end_moves(self, robot):
        m = self.moves.pop(robot, None)
        if not m or m["run"] == "-":
            return
        self.send(robot, "mark end %d" % self.next_id(robot))
        if self.watch.wait_end(m["run"], 6) is None:
            self.send(robot, "stop")
            self.watch.wait_end(m["run"], 3)

    def next_id(self, robot):
        """Leg ids only ever go up (the firmware answers `dup` to an id it has seen: a resend is safe)."""
        self.leg_ids[robot] = self.leg_ids.get(robot, int(time.time()) % 100000 * 10) + 1
        return self.leg_ids[robot]

    def send_leg(self, robot, text, timeout):
        """One leg or mark with the next id, resent once (same id) if no answer. Returns (state, info)."""
        m = self.moves.get(robot)
        if m is None:
            return "refused", "not moving"
        if robot in self.held_out:
            return "refused", "left out by Nick"
        verb = text.split()[0]
        if verb == "turn" and m["run"] == "-":
            timeout = self.turn_speed_for(robot, timeout)     # fix 5: slower near an open edge
        lid = self.next_id(robot)
        self.last_lid[robot] = lid
        following = self.tracker is not None and self.tracker.active == robot
        if following:
            self.tracker.clear_expect()          # it may move now
            self.expect_leg(text, timeout)
        if verb == "turn":
            w_ = self.tracker.where() if following else None
            self.__dict__.setdefault("turn_start_xy", {})[robot] = (w_[1], w_[2]) if w_ else None
        try:
            out = self._send_leg_once(robot, text, timeout, m, lid, following)
        finally:
            if verb == "go":
                self.__dict__.setdefault("go_end", {})[robot] = time.time()
        self.leg_health(robot, text, m, lid, out)
        return out

    def turn_speed_for(self, robot, timeout):
        """Fix 5: an idle turn within TURN_SLOW_ROOM_CM of an open edge goes at TURN_SLOW_RPM (and gets
        TURN_SLOW_TIMEOUT_MS); elsewhere at full speed. Sent only when it changes. Returns the leg's timeout (s),
        longer for a slow turn."""
        room = self.open_room_now(robot)
        slow = room is not None and room < TURN_SLOW_ROOM_CM
        rpm = TURN_SLOW_RPM if slow else TURN_FAST_RPM
        tsn = self.__dict__.setdefault("turn_speed_now", {})
        if tsn.get(robot, TURN_FAST_RPM) != rpm:
            self.link.send_name(robot, "set turn_max_rpm %g" % rpm)
            time.sleep(0.05)
            self.link.send_name(robot, "set turn_timeout_ms %d" % (TURN_SLOW_TIMEOUT_MS if slow else TURN_TIMEOUT_MS))
            time.sleep(0.05)
            tsn[robot] = rpm
            self.say("    %s: turns %s (%s)" % (robot, "slower" if slow else "at full speed again",
                                                  "%.0f cm from an open edge" % room if room is not None else "-"),
                     quiet=True)
        if slow:
            self.st.setdefault("slow_turns", []).append([round(time.time(), 1), robot, round(room, 1)])
            return timeout * TURN_FAST_RPM / TURN_SLOW_RPM
        return timeout

    def open_room_now(self, robot):
        """Signed distance (cm) from the robot's centre to the nearest open edge (- past it): the camera's latest
        position when it follows this robot, else its parked spot; None if neither or no board."""
        bd = self.board
        if bd is None or getattr(self, "exploring", False):
            return None
        q = None
        if self.tracker is not None and self.tracker.active == robot:
            w_ = self.tracker.where()
            if w_ and time.time() - w_[0] < STALE_S:
                q = (w_[1], w_[2])
        if q is None and (self.st.get("positions") or {}).get(robot):
            q = tuple(self.st["positions"][robot][:2])
        if q is None:
            return None
        return min(ST.inward_cm(bd, a, v, q[0] if a == "x" else q[1]) for a, v in bd.open_edges())

    def leg_health(self, robot, text, m, lid, out):
        """After every leg: fix 1's turn health (a robot whose turns keep failing is stopped, named and left out),
        the robot's own heading change over a turn (fix 3: what a cut turn did), and when its own edge guard
        stopped it (fix 4: the back-out after it is allowed)."""
        state, info = out
        verb = text.split()[0]
        if "tilted" in (info or "") and time.time() - (getattr(self, "tilt_said", None) or {}).get(robot, 0) > 30 \
                and robot not in self.held_out:
            # item 4: its own tilt stop (firmware 2026-10-26a) stopped or refused the leg
            self.__dict__.setdefault("tilt_said", {})[robot] = time.time()
            self.hung_stop(robot, "tilt", "its own tilt stop: %s" % info)
        if "plywood edge" in (info or ""):
            self.__dict__.setdefault("edge_stop_t", {})[robot] = time.time()
            self.__dict__.setdefault("backed_since_edge", {})[robot] = 0.0
        elif state in ("done", "failed") and (verb == "turn" or (verb == "go" and (num(text.split()[1]) or 0) > 0)):
            # a turn or forward drive really ran: the next back-out is no longer "right after" the edge stop
            (getattr(self, "edge_stop_t", None) or {}).pop(robot, None)
            (getattr(self, "backed_since_edge", None) or {}).pop(robot, None)
        elif state in ("done", "failed") and verb == "go" and robot in (getattr(self, "edge_stop_t", None) or {}):
            # backing out after its edge stop: how far (its own wheels; the leg's value if not known)
            m = self.moves.get(robot) or {}
            lid_ = self.last_lid.get(robot)
            st0 = self.listener.leg_start(robot, m.get("run", "-"), lid_) if lid_ is not None else None
            fin = self.listener.leg_final(robot, m.get("run", "-"), lid_) if lid_ is not None else None
            dl = leg_delta(st0, fin) if st0 is not None and fin is not None else (None, None, None)
            # the straight distance (its own frame, or the board's once a camera fix set its pose: both axes; dry run
            # D1, 2026-10-26: x alone read 5.3 cm for a 9.5 cm back-out at a slant)
            moved = math.hypot(dl[0], dl[1]) if None not in dl[:2] else \
                (abs(num(text.split()[1]) or 0.0) if state == "done" else 0.0)
            bse = self.__dict__.setdefault("backed_since_edge", {})
            bse[robot] = bse.get(robot, 0.0) + moved
        if verb not in ("go", "turn") or getattr(self, "listener", None) is None:
            return
        tw = getattr(self, "turnwatch", None)
        if verb == "go":
            if tw is not None and state not in ("refused", "dup", "lost"):
                tw.feed_go(robot)
            return
        val = num(text.split()[1]) if len(text.split()) > 1 else None
        st0 = self.listener.leg_start(robot, m["run"], lid)
        fin = self.listener.leg_final(robot, m["run"], lid)
        done = None
        if st0 is not None and fin is not None:
            done = TW.turn_done_deg(num(st0.get("th_deg")), num(fin.get("th_deg")))
        self.__dict__.setdefault("turn_done", {})[robot] = done
        if state in ("done", "failed", "lost"):
            self.__dict__.setdefault("turned_since_measure", {})[robot] = True
        if tw is None or val is None or state == "lost":
            return
        why = tw.feed_turn(robot, val, state, info, done)
        if why:
            self.turn_fault_found(robot, why)

    def turn_fault_found(self, robot, why):
        """Fix 1: a robot whose turns keep failing is stopped, named, and left out for the rest of the session (Nick,
        2026-10-25 18:46: "leave that robot out and carry on with the others"); the live page sounds. Nobody knows
        where it is (no camera position, no parked spot): Nick is asked instead (a hold), so the others cannot
        drive into it."""
        for _ in range(3):
            try:
                self.link.send_name(robot, "stop")
            except Exception:
                pass
            time.sleep(0.1)
        rep = self.__dict__.setdefault("compass_repairs", {})
        if not rep.get(robot) and not getattr(self, "in_compass", False) and getattr(self, "watch", None) is not None:
            # the 15:45 root cause: a compass check that learned the wrong sign. A new compass check, verified by a
            # check turn, mends that: tried once before the robot is left out
            rep[robot] = True
            self.say("!!! TURNS: %s's turns are failing (%s): checking its compass again before anything else" % (
                robot, why))
            self.st.setdefault("compass_repairs", []).append([round(time.time(), 1), robot, why])
            self.in_compass = True
            had = self.moves.get(robot)
            try:
                self._calibrate([robot], verify=False)
                ok, how = self.compass_turn_check(robot)
            finally:
                self.in_compass = False
                if had is not None:
                    self.moves[robot] = had          # (the compass check ends its idle moves)
            self.st["compass_repairs"][-1] += [ok, how]
            if ok:
                self.turnwatch.tripped.pop(robot, None)
                self.turnwatch.run[robot] = 0
                self.turnwatch.corr.pop(robot, None)
                self.alert("compass", robot, "%s's turns failed; a new compass check mended them" % robot)
                self.say("  %s: its new compass check is good (%s): it carries on" % (robot, how))
                return
        words = "%s's turns are not working: %s" % (robot, why)
        self.st.setdefault("turn_faults", []).append([round(time.time(), 1), robot, why])
        self.say("!!! TURNS: %s. It is stopped and left out for the rest of this session; the others carry on. Check "
                 "it on the bench (turn 90 on the table) before the next session." % words)
        self.mark("e02 turn fault %s" % robot)
        self.alert("turn_fault", robot, words + ": stopped and left out")
        if not self.leave_out(robot):
            # the camera does not know where it is yet (its turns failed while it was being found): it is left out
            # all the same (nothing drives it again), and where its lights are found is kept on the map so the others
            # go round it (identify). 19:03 dry run: a hold here, answered "carry on", let it be driven on.
            self.held_out.add(robot)
            self.unavailable.add(robot)
            self.moves.pop(robot, None)
            self.st.setdefault("left_out", []).append([round(time.time(), 1), robot, None])
            self.say("  %s: the camera does not know where it is yet: it is found by its lights only (no turns), "
                     "and the others keep clear of where it is found." % robot)
            if getattr(self, "feed", None) is not None and getattr(self, "cam", None) is not None \
                    and not getattr(self, "in_identify", False):
                try:
                    p_ = self.identify(robot)       # lights only (it is left out): kept on the map where found
                except Exception as e:
                    p_ = None
                    self.say("  (%s: not found by its lights: %s)" % (robot, e), quiet=True)
                if p_ is None:
                    self.say("!!! %s is left out and the camera cannot see where it is: keep the others clear of it by "
                             "hand, or take it off the board." % robot)
                    self.alert("help", robot, "%s is left out and the camera cannot find it" % robot)
                else:
                    self.say("  %s is at (%.1f, %.1f) cm (found by its lights): the others go round it." % (
                        robot, p_[0], p_[1]))
        self.st.save()

    def alert(self, kind, robot, words):
        """Something Nick must hear (the live page plays a sound for each new one): a hold, a help call, a robot
        left out for a fault. Kept in the state (alerts)."""
        al = self.st.setdefault("alerts", [])
        al.append(dict(id=len(al) + 1, t=round(time.time(), 1), kind=kind, robot=robot, words=words))

    def _send_leg_once(self, robot, text, timeout, m, lid, following):
        for attempt in range(2):
            t_s = time.time()
            if self.send(robot, "%s %d" % (text, lid)) is False:
                return "refused", "left out by Nick"
            if not text.startswith("mark"):
                self.__dict__.setdefault("leg_text", {})[robot] = "%s %d" % (text, lid)
                self.leg_live[robot] = time.time()
                self.leg_kind[robot] = text.split()[0]
                self.leg_val[robot] = num(text.split()[1]) if len(text.split()) > 1 else None
            stop = (lambda: self.watch.ended.get(m["run"]) is not None) if m["run"] != "-" else None
            try:
                res = self.listener.wait_leg(robot, m["run"], lid, t_s - 0.05, timeout, stop=stop)
            finally:
                self.leg_live.pop(robot, None)
                self.leg_end[robot] = time.time()
            if res and following and res.get("state") in ("done", "failed", "dup"):
                time.sleep(0.3)                  # it has stopped: from now on it can't be anywhere else
                self.tracker.hold(HOLD_CM)
            if res:
                if res.get("state") == "refused" and "running" in (res.get("info") or "") and attempt == 0:
                    time.sleep(1.0)              # an idle leg still finishing: once more
                    continue
                if res.get("state") == "refused" and "no compass check" in (res.get("info") or "") and attempt == 0 \
                        and not getattr(self, "in_compass", False):
                    # firmware 2026-10-25c: no compass check since power-on (switched off and on, a restart): the
                    # compass check now, verified, then the leg once more (coordinator, 2026-10-25 19:20)
                    self.say("!!! COMPASS: %s says it has had no compass check since power-on: checking it now" % robot)
                    self.st.setdefault("compass_missing", []).append([round(time.time(), 1), robot, text])
                    self.in_compass = True
                    try:
                        self._calibrate([robot])
                    finally:
                        self.in_compass = False
                        self.moves[robot] = m        # (the compass check ends its idle moves)
                    if robot in self.held_out:
                        return "refused", "left out: its compass check failed"
                    lid = self.next_id(robot)
                    self.last_lid[robot] = lid
                    continue
                return res.get("state", "?"), res.get("info", "")
            if m["run"] != "-" and self.watch.ended.get(m["run"]) is not None:
                self.moves.pop(robot, None)
                return "refused", "its camera-steered run ended (%s)" % self.watch.ended[m["run"]]["result"]
        return "lost", "no answer"

    def unc_cm(self, robot, extra_go=0.0, extra_turn=0.0):
        """The robot's own position uncertainty (1 sd, cm) since its last camera fix or find (item 4), by its
        learned model (e02_unc)."""
        u = self.unc.setdefault(robot, dict(dist=0.0, turned=0.0))
        d, t = u["dist"] + abs(extra_go), u["turned"] + abs(extra_turn)
        return float(UNC.sd_of(self.unc_params(robot), d, t))

    def unc_params(self, robot):
        """(base, per_cm, deg, deg_per_deg) of the uncertainty model in use for this robot: learned (e02_unc), or the
        fixed starting values with --no-unc-learn."""
        um = getattr(self, "uncm", None)
        if um is None or getattr(getattr(self, "a", None), "no_unc_learn", False):
            return (UNC_BASE_CM, UNC_PER_CM, UNC_DEG, UNC_DEG_PER_DEG)
        return um.params(robot)

    def unc_reset(self, robot):
        # clean: nothing but idle legs since (a run's own driving is not counted here, so a correction after one
        # says nothing about this model: unc_learn leaves it out)
        self.unc[robot] = dict(dist=0.0, turned=0.0, clean=True)

    def unc_learn(self, robot, own, cam, kind="fix", dist=None, turned=None):
        """One camera correction for the uncertainty model (e02_unc): the robot's own position `own` just before the
        camera's `cam` replaced it, with what it drove and turned since its last fix (this robot's unc, or given).
        Logged (unc_obs.csv); a changed robot is said loudly. Never stops the session."""
        um = getattr(self, "uncm", None)
        if um is None or own is None or cam is None:
            return
        try:
            if dist is None:
                u = self.unc.get(robot) or {}
                if not u.get("clean") or (u.get("dist", 0.0) <= 0.0):
                    return                       # a run since the last reset, or nothing driven: not this model's
                dist, turned = u["dist"], u.get("turned", 0.0)
            corr = math.hypot(cam[0] - own[0], cam[1] - own[1])
            ce_ = getattr(self, "camerr", None)
            csd = ce_.corrected(cam[0], cam[1])[2] if ce_ is not None else None
            words = um.add(robot, dist, turned or 0.0, corr, csd, kind=kind)
            path = os.path.join(self.dir, "unc_obs.csv")
            new = not os.path.exists(path)
            with open(path, "a", newline="") as f:
                w = csv.writer(f)
                if new:
                    w.writerow(["t", "robot", "kind", "dist_cm", "turned_deg", "correction_cm", "cam_sd_cm",
                                "sd_model_cm", "model_from"])
                m_ = um.model(robot)
                w.writerow([round(time.time(), 2), robot, kind, round(dist, 1), round(turned or 0.0, 1),
                            round(corr, 2), "" if csd is None else round(csd, 2),
                            round(float(UNC.sd_of(m_["p"], dist, turned or 0.0)), 2), m_["src"]])
            if words:
                self.say("!!! UNCERTAINTY: " + words + ".")
                self.mark("e02 uncertainty changed %s" % robot)
                self.st.setdefault("unc_changes", []).append([round(time.time(), 1), robot, words])
        except Exception as e:
            self.say("  (uncertainty correction not learned: %s)" % e, quiet=True)

    def leg_pause_ms(self):
        """The stand-still after every leg of a mode-0 run (all conditions alike), long enough for a KC camera fix
        to land: the firmware's still check + still pictures + this session's picture delay (95th percentile of the
        last minutes' worst age a second) + a margin. 2026-10-24 review: at a fixed 700 ms two thirds of KC's fixes
        arrived after the robot had moved on."""
        rows = (getattr(self, "perf_rows", None) or [])[-300:]
        ages = sorted(r.get("age_max_s", 0.0) for r in rows if r.get("age_max_s") is not None)
        p95 = ages[int(0.95 * (len(ages) - 1))] if ages else 0.3
        p95 = min(p95, 1.5)                       # a WiFi copy's seconds-long stalls: pausing longer would not help
        ms = 300 + 1000.0 * (KC_STILL_S + p95 + KC_SEND_MARGIN_S)
        ms = int(round(min(LEG_PAUSE_MAX_MS, max(LEG_PAUSE_MIN_MS, ms)) / 100.0) * 100)
        st = getattr(self, "st", None)
        if st is not None:
            st.setdefault("leg_pause_ms", []).append([round(time.time(), 1), ms, round(p95, 3)])
        return ms

    def trust_note(self, save=True):
        """Say loudly when a sensor starts or stops looking bad (mark on the video too); save the learned trust
        (not after every leg's front check: 2026-10-24 review, the Pi's card)."""
        names = [n for n in self.st["robots"]]
        for s_, r_, what, kk in self.trust.update_flags(names):
            who = "the camera" if r_ == "*" else "%s's %s" % (r_, {"front": "front sensor"}.get(s_, s_))
            if what == "degraded":
                rsn = self.trust.reason(s_, r_) if hasattr(self.trust, "reason") else ""
                self.say("!!! SENSOR: %s looks bad: %.1f x its own baseline (e02_trust%s). It is trusted less from "
                         "now on." % (who, kk, (": " + rsn) if rsn else ""))
            else:
                self.say("  SENSOR: %s looks healthy again (%.1f x its own baseline)." % (who, kk))
            self.mark("e02 sensor %s %s %s" % (s_, r_, what))
            if s_ == "camera":
                self.say("  Working area: the virtual border is now %.0f cm inside the open edges%s." % (
                    self.border_cm(), " (the camera looks bad: the area shrinks)" if what == "degraded" else ""))
                self.st.setdefault("border_changes", []).append([round(time.time(), 1), round(self.border_cm(), 1)])
                if self.board is not None:
                    bad_ = what == "degraded"
                    self.board.park_min_open_cm = min(self.border_cm(), PARK_OPEN_MAX_CM) if bad_ else 0.0
                    self.say("  Park spots: %s." % ("moved in to %.0f cm from the open edges (each robot goes there "
                                                    "when it next parks)" % self.board.park_min_open_cm if bad_ else
                                                    "back where they were"))
            self.st.setdefault("sensor_trust_events", []).append([round(time.time(), 1), s_, r_, what, round(kk, 2)])
        if not save:
            return
        try:
            TR.save(self.trust, self.trust_path)
        except OSError as e:
            self.say("  (sensor trust not saved: %s)" % e, quiet=True)

    def camerr_add(self, x, y, cam, source, robot):
        """One place where the camera's error is known (e02_camerr.py), saved in the state."""
        if cam is None or x is None or y is None:
            return None
        smp = self.camerr.add(x, y, cam[0], cam[1], source, robot=robot, t=round(time.time(), 1))
        self.st["camera_error_samples"] = self.camerr.to_list()
        return smp

    def pictures_late(self):
        """Item 5: is the camera's newest picture older than LATE_PICTURE_S (or the last second's oldest)?"""
        f = self.feed
        t = getattr(f, "last_t", None) if f is not None else None
        if t is not None and time.time() - t > LATE_PICTURE_S:
            return True
        rows = getattr(self, "perf_rows", None) or []
        return bool(rows) and time.time() - rows[-1].get("t", 0) < 2.0 and rows[-1].get("age_max_s", 0) > LATE_PICTURE_S

    def confidence_pause(self, robot, kind, val):
        """Item 4: before a drive that would take its uncertainty past CONF_PAUSE_CM, stop and take a fresh camera
        fix (it stands still for it). With the pictures late (item 5), no pause: it drives on its own sensors, and
        the boards correct it. Every decision is logged (confidence_pauses in the state, confidence.csv)."""
        if kind != "go":
            return
        now_sd, after_sd = self.unc_cm(robot), self.unc_cm(robot, extra_go=val)
        u0 = dict(self.unc.get(robot) or {})
        if after_sd <= CONF_PAUSE_CM:
            return
        late = self.pictures_late()
        own = self.own_pose(robot)
        p = None if late else self.fix_now(robot)
        corr = None if (p is None or own is None) else round(math.hypot(p[0] - own[0], p[1] - own[1]), 2)
        what = "paused for a camera fix" if p is not None else (
            "pictures late: drove on its own sensors" if late else "paused, but the camera did not see it")
        if p is not None:
            self.unc_reset(robot)
        row = [round(time.time(), 2), robot, round(now_sd, 2), round(after_sd, 2), CONF_PAUSE_CM, what,
               self.order or "", "" if corr is None else corr, round(u0.get("dist", 0.0), 1),
               round(u0.get("turned", 0.0), 1)]
        self.st.setdefault("confidence_pauses", []).append(row)
        path = os.path.join(self.dir, "confidence.csv")
        new = not os.path.exists(path)
        with open(path, "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["t", "robot", "sd_now_cm", "sd_after_cm", "threshold_cm", "what", "order",
                            "correction_cm", "dist_cm", "turned_deg"])
            w.writerow(row)
        self.say("    %s: %s (its position uncertainty would reach %.1f cm, over %.0f)" % (
            robot, what, after_sd, CONF_PAUSE_CM), quiet=True)

    def edge_touch_log(self, robot, p, words, kind):
        """Where an open edge stopped a robot (its edge guard), against the map's open edge: the edge's felt position
        (the camera's position plus the cue's reach along its heading) for the edge nearest. Logged only for now
        (edge_touches in the state, said in the session summary), and the open edges move to where the robots feel
        them like the boards (learn_open_edges, 2026-10-24: the night of 2026-10-23 gave the first real edge
        stops)."""
        if kind != "go" or p is None or "plywood edge" not in (words or "") or self.board is None:
            return
        h, meas = (list(self.heads.get(robot) or [None, False]) + [False])[:2]
        if h is None:
            return
        if not meas:
            # fix 4 (15:45): Left's three touches logged with a heading it did not have (refused turns counted as
            # done) moved the bottom line 8 cm in. Only a heading measured since its last turn places a touch.
            self.st.setdefault("edge_touches_skipped", []).append([round(time.time(), 1), robot, MAP.edge_cue(words)])
            return
        fx, fy = MAP.contact_point(p[0], p[1], h, "edge", MAP.edge_cue(words))
        a, v = min(self.board.open_edges(), key=lambda e: abs((fx if e[0] == "x" else fy) - e[1]))
        felt = fx if a == "x" else fy
        if abs(felt - v) > 15.0:
            return
        self.st.setdefault("edge_touches", []).append(
            [round(time.time(), 1), robot, a, round(v, 2), round(felt, 2), MAP.edge_cue(words)])
        self.learn_open_edges()

    EDGE_LEARN_MIN_N = 3          # chosen, as for the boards: three edge stops before an open edge moves
    EDGE_LEARN_STEP_CM = 0.5
    EDGE_LEARN_IN_CM = 8.0        # chosen: an open edge may move inwards (safer) up to this from its first line
    EDGE_LEARN_OUT_CM = 3.0       # ... and outwards (less safe: the watchdog lets robots nearer) only this far.
    # DOE for both caps: edge stops at known tape distances from the plywood's end (the crawl-speed experiment's runs)

    def learn_open_edges(self):
        """The map's open edges move to where the robots' edge guards feel them (the median of the last touches of
        that edge), after EDGE_LEARN_MIN_N touches, inwards up to EDGE_LEARN_IN_CM and outwards up to
        EDGE_LEARN_OUT_CM from where the edge first stood; logged (open_edge_updates) and said."""
        st, bd = self.st, self.board
        first = st.setdefault("open_edges_first", {})
        for a, v in bd.open_edges():
            attr = ("x0" if abs(v - bd.x0) < 1e-6 else "x1") if a == "x" else ("y0" if abs(v - bd.y0) < 1e-6 else "y1")
            rows = [r for r in st.get("edge_touches") or [] if r[2] == a and abs(r[4] - v) <= 15.0][-12:]
            if len(rows) < self.EDGE_LEARN_MIN_N:
                continue
            v0 = first.setdefault(attr, round(v, 2))
            inward = 1.0 if attr.endswith("0") else -1.0          # x0 / y0 edges: inside is larger values
            felt = float(np.median([r[4] for r in rows]))
            move = (felt - v0) * inward                          # + = inwards
            move = min(self.EDGE_LEARN_IN_CM, max(-self.EDGE_LEARN_OUT_CM, move))
            line = v0 + inward * move
            if abs(line - v) < self.EDGE_LEARN_STEP_CM:
                continue
            setattr(bd, attr, line)
            st.setdefault("open_edge_updates", []).append(dict(t=round(time.time(), 1), edge=attr, before=round(v, 2),
                                                               after=round(line, 2), first=v0, touches=len(rows)))
            self.say("    the open edge %s moved %+.1f cm to where the robots' edge guards feel it (%.2f cm; %d stops)"
                     % (attr, line - v, line, len(rows)))
            self.mark("e02 open edge %s %.2f" % (attr, line))
            st["board"] = bd.to_dict()
            self.tracker_area()
            st.save()

    def board_correct(self, robot, front, ahead, words):
        """Item 5: pictures late, the front sensor reading a board square on: its position along that board's axis
        is where the board line and the reading put it; the other coordinate and the heading stay its own. Sent as
        a fix; logged (board_corrections)."""
        own = self.own_pose(robot)
        if own is None or ahead is None or "board" not in str(ahead[1]):
            return None
        off, tol = self.front_limits(robot)
        hd = own[2]
        ux, uy = math.cos(math.radians(hd)), math.sin(math.radians(hd))
        reach = front - off + PL.ROBOT_HALF_CM            # axle to the board along its heading
        x, y = own[0], own[1]
        if ahead[1] == "the y-board" and abs(ux) >= FRONT_MIN_COS:
            x = self.board.yboard_cm - reach * ux
        elif ahead[1] == "the x-board" and abs(uy) >= FRONT_MIN_COS:
            y = self.board.xboard_cm - reach * uy
        else:
            return None
        shift = math.hypot(x - own[0], y - own[1])
        if shift > 15.0:
            return None                                   # too far to be a correction: something else is ahead
        self.send(robot, "fix %.2f %.2f %.1f %.3f" % (x, y, hd, time.time()))
        self.unc_reset(robot)
        self.st.setdefault("board_corrections", []).append(
            [round(time.time(), 2), robot, ahead[1], round(front, 1), [round(v, 1) for v in own], [round(x, 1), round(y, 1)],
             round(shift, 2)])
        self.say("    %s: pictures late; its front sensor on %s corrected it %.1f cm" % (robot, ahead[1], shift),
                 quiet=True)
        return (x, y)

    def front_check(self, robot, p, words, kind, val):
        """Nick, 2026-10-22: the front sensor as a free check of the position. After every finished leg the firmware
        (2026-10-22b) reports the front sensor's middle zone ("front=<cm>"); here it is compared with what the camera
        position, the heading and the map say is ahead (a board line, or another robot), and written to
        front_checks.csv. Only recorded for now (the numbers from a session decide the limits before it steers
        anything); a large disagreement is said quietly."""
        m = re.search(r"front=([0-9.]+)", words or "")
        h = self.heads.get(robot, [None, False])
        if not m or p is None or h[0] is None or self.board is None:
            return None
        front = float(m.group(1))
        if front <= 0:
            return None                           # no reading (nothing in range)
        if self.degrade.spec.get("front"):
            front = self.degrade.front(front, robot)     # --degrade: a bad front sensor
        hd = ST.wrap180(h[0] + val) if kind == "turn" else h[0]      # heads[] holds the heading before the leg
        ahead = self.ahead_cm(robot, p, hd)
        row = [round(time.time(), 2), robot, kind, round(val, 1), round(p[0], 1), round(p[1], 1), round(hd, 1),
               int(bool(h[1])), front, "" if ahead is None else round(ahead[0], 1), "" if ahead is None else ahead[1]]
        path = os.path.join(self.dir, "front_checks.csv")
        new = not os.path.exists(path)
        with open(path, "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["t", "robot", "leg", "value", "x_cm", "y_cm", "heading_deg", "heading_measured",
                            "front_cm", "expected_cm", "what"])
            w.writerow(row)
        if ahead is not None and ahead[0] <= FRONT_CHECK_RANGE_CM and self.square_on(ahead):
            off, tol = self.front_limits(robot)
            miss = front - off - ahead[0]
            if h[1]:
                self.front_seen.setdefault("all", []).append(front - ahead[0])
                fm = getattr(self, "front", None)
                if fm is not None:
                    ch_ = fm.add(robot, front - ahead[0])
                    if ch_:
                        self.say("!!! FRONT SENSOR: " + ch_ + ".")
                        self.mark("e02 front sensor changed %s" % robot)
                        self.st.setdefault("front_changes", []).append([round(time.time(), 1), robot, ch_])
                        if hasattr(self.trust, "mark_changed"):
                            self.trust.mark_changed("front", robot, ch_)    # a clear change flags it (Nick, 10-25)
                self.trust.front_vs_map(robot, miss)      # after the expected offset: a bias counts too
                self.trust_note(save=False)
            if abs(miss) > tol:
                self.say("    %s: front sensor %.0f cm, the camera and map say %s at %.0f cm" % (
                    robot, front, ahead[1], ahead[0]), quiet=not (h[1] and abs(miss) > FRONT_STEER_FACTOR * tol))
                if h[1] and abs(miss) > FRONT_STEER_FACTOR * tol and not getattr(self.a, "no_front_steer", False) \
                        and not self.trust.bad("front", robot) and not self.trust.bad("camera"):
                    # (nor while the camera itself looks bad: the map's position is known to be off, and a hold
                    # would only say so again; 2026-10-25 simulator, camera noise 4 cm: 6 front-sensor holds)
                    # (a front sensor that has itself been found bad does not make the camera suspect: e02_trust)
                    # a measured heading, and the sensor far from the map: the camera position is suspect
                    self.front_bad[robot] = "the front sensor reads %.0f cm, the map says %s at %.0f cm" % (
                        front, ahead[1], ahead[0] + off)
        return front, ahead

    def front_limits(self, robot=None):
        """(offset, tolerance) cm for the front sensor against the map. Nick, 2026-10-22: for now from data gathered
        before the session (FRONT_OFFSET_CM / FRONT_TOL_CM, to be set from the front_checks.csv of the next real
        session); --front-learn (off by default) learns them in this session instead, from scratch, once
        FRONT_LEARN_MIN readings are in: the median of reading - map, and 3 x its spread (MAD x 1.48), at least
        FRONT_LEARN_MIN_TOL. NOTE (Nick): switch to --front-learn once the from-scratch setup can stand on it."""
        fm = getattr(self, "front", None)
        if robot is not None and fm is not None and not getattr(self.a, "no_front_learn", False):
            off_, tol_, _src = fm.limits(robot)       # each robot's own, learned every session (Nick, 2026-10-25)
            return off_, tol_
        v = self.front_seen.get("all") or []
        if getattr(self.a, "front_learn", False) and len(v) >= FRONT_LEARN_MIN:
            med = float(np.median(v))
            mad = float(np.median([abs(x - med) for x in v])) * 1.4826
            return med, max(FRONT_LEARN_MIN_TOL, 3.0 * mad)
        return FRONT_OFFSET_CM, FRONT_TOL_CM

    def ahead_cm(self, robot, p, heading):
        """Distance from the robot's front face to the first thing straight ahead, by the map: (cm, what) or None.
        Boards are lines; other robots are discs of PL.ROBOT_HALF_CM."""
        b = self.board
        ux, uy = math.cos(math.radians(heading)), math.sin(math.radians(heading))
        best = None
        gone = getattr(b, "missing", ())
        for what, axis, v in (("the y-board", "x", b.yboard_cm), ("the x-board", "y", b.xboard_cm)):
            if what[4] in gone:
                continue                      # a board found gone (or --no-boards): nothing there to read
            d = (v - p[0]) / ux if axis == "x" and abs(ux) > 1e-6 else ((v - p[1]) / uy if axis == "y" and abs(uy) > 1e-6
                                                                      else -1)
            if d > 0 and (best is None or d < best[0]):
                best = (d, what, abs(ux) if axis == "x" else abs(uy))     # cos of the angle to the board's normal
        for n, q in self.st["positions"].items():
            if n == robot or not q:
                continue
            ox, oy = q[0] - p[0], q[1] - p[1]
            t = ox * ux + oy * uy
            off = abs(-ox * uy + oy * ux)
            if t > 0 and off < PL.ROBOT_HALF_CM:
                d = t - math.sqrt(max(0.0, PL.ROBOT_HALF_CM ** 2 - off ** 2))
                if best is None or d < best[0]:
                    best = (d, n, 1.0)
        if best is None:
            return None
        return best[0] - PL.ROBOT_HALF_CM, best[1], best[2]

    @staticmethod
    def square_on(ahead):
        """Is the thing ahead met steeply enough for the front sensor to read it? (dry run 15:35: at 19 deg to a
        board the sensor read 92 cm where the board was 42 cm away; its zones see past a board met that flat)"""
        return ahead is not None and (len(ahead) < 3 or ahead[2] >= FRONT_MIN_COS)

    def expect_leg(self, text, timeout):
        """Where the followed robot can be during this leg (2026-10-22: during a turn on its spot the tracker was
        free to take another robot 34 cm away): a turn keeps it within EXPECT_TURN_CM of where it stands; a drive
        of d cm within d + EXPECT_GO_EXTRA_CM. A camera-steered run (no tracker.where()) is left free."""
        p = self.tracker.where()
        if p is None:
            return
        parts = text.split()
        try:
            verb, val = parts[0], abs(float(parts[1]))
        except (IndexError, ValueError):
            return
        if verb == "turn":
            r = EXPECT_TURN_CM
        elif verb in ("go", "probe"):
            r = val + EXPECT_GO_EXTRA_CM
        else:
            return
        self.tracker.expect(p[1], p[2], r, for_s=timeout + 3.0, kind="turn" if verb == "turn" else "go")

    def room_behind(self, robot, p, back_cm):
        """Backing up is blind (no edge guard, no front sensor): is there room behind for back_cm, by the camera?
        Returns plain words of what is in the way, or ''. Unknown heading: assume the worst way it could face."""
        h = self.heads.get(robot, [None])[0]
        if p is None:
            return "the camera can't see it"
        heads = [h] if h is not None else list(range(0, 360, 30))
        parked = {n: tuple(q[:2]) for n, q in self.st["positions"].items() if n != robot and q}
        for hh in heads:
            ux, uy = math.cos(math.radians(hh)), math.sin(math.radians(hh))
            for f in (0.5, 1.0):
                cx, cy = p[0] - ux * back_cm * f, p[1] - uy * back_cm * f
                rx, ry = cx - ux * PL.ROBOT_HALF_CM, cy - uy * PL.ROBOT_HALF_CM
                if not self.board.on_board(rx, ry, margin=2.0):
                    return "an open edge behind it"
                for n, q in parked.items():
                    if math.hypot(cx - q[0], cy - q[1]) < PL.WATCH_ROBOT_CM:
                        return "%s parked behind it" % n
        return ""

    def leg(self, robot, cmd, run=None, backout=False):
        """One go / turn leg. A backwards go goes in pieces of at most BACK_MAX_CM; before each the camera checks
        the room behind it and a camera fix is sent (the firmware allows e02_back_max_cm of blind backing per
        fix). backout: a straight back-out right after the robot's own edge guard stopped it (fix 4): the way it
        came is clear, so no room check (with its heading unknown that check assumes every way, and near an edge
        always finds "an open edge behind it"). Returns (p0, p1, words)."""
        kind, val = ("go" if cmd[0] == "probe" else cmd[0]), float(cmd[1])
        if getattr(self, "scene", None):
            self.dry_scene_leg(robot, kind, val, ctx="leg")
        lid_before = self.last_lid.get(robot)
        p0 = self.settle(min_s=0.3, timeout=1.5)
        pieces = [val]
        if kind == "go" and val < -BACK_MAX_CM:
            n = int(math.ceil(-val / BACK_MAX_CM))
            pieces = [val / n] * n
        words = ""
        self.watch_stop = None
        for v in pieces:
            if kind == "go" and v < 0:
                pb = self.settle(min_s=0.3, timeout=1.5)
                bad = "" if backout else self.room_behind(robot, pb, -v)
                if bad:
                    words = "refused (no room to back up: %s)" % bad
                    break
                self.fix_now(robot)
            if robot in self.moves:
                self.confidence_pause(robot, kind, v)
            for attempt in range(2):
                if robot not in self.moves and not self.begin_moves(robot):
                    return p0, p0, "could not start camera-steered moves"
                est = abs(v) / DRIVE_CMS_GUESS + 1.0 if kind == "go" else abs(v) / TURN_DPS_GUESS + 1.0
                state, info = self.send_leg(robot, "%s %.2f" % (kind, v), 2.5 * est + 6.0)
                words = state + ((" (" + info + ")") if info else "")
                if state == "refused" and robot not in self.moves and attempt == 0:
                    continue                      # its run had ended (quiet too long): start another, try again
                if state == "refused" and "busy" in info and attempt == 0:
                    time.sleep(1.5)               # its run is still starting (settle pause, start flash)
                    continue
                if state == "refused" and "backing blind" in info and attempt == 0:
                    self.fix_now(robot)           # the fix did not arrive: once more
                    continue
                break
            if self.watch_stop:
                words = "watchdog (%s)" % self.watch_stop
                break
            if state not in ("done", "dup"):
                break
        time.sleep(0.25)
        self.main_lid[robot] = self.last_lid.get(robot)
        u = self.unc.setdefault(robot, dict(dist=0.0, turned=0.0))
        if kind == "go":
            u["dist"] += abs(val)
        else:
            u["turned"] += abs(val)
        p1 = self.settle(t_from=time.time() - 0.6, min_s=0.3, timeout=2.0)
        if p1 is None and self.pictures_late():
            own = self.own_pose(robot)
            fc = self.front_check(robot, own[:2], words, kind, val) if own else None
            if fc and fc[1] is not None and fc[1][0] <= FRONT_CHECK_RANGE_CM and self.square_on(fc[1]):
                q = self.board_correct(robot, fc[0], fc[1], words)
                if q is not None:
                    p1 = q
            return p0, p1, words
        self.front_check(robot, p1, words, kind, val)
        self.edge_touch_log(robot, p1, words, kind)
        # only a leg that really ran is checked: a refused one leaves last_lid at the leg before, whose wheel
        # distance would be compared again (2026-10-22 20:00: a back-off refused at the open edge -> refind ->
        # back-off refused -> ... "the camera saw it move 0 cm, its wheels say 72 cm" once a second, forever)
        ran = self.last_lid.get(robot) != lid_before
        if kind == "go" and p0 is not None and len(pieces) == 1 and ran and robot not in self.refinding:
            bad = self.cross_check(robot, p0, p1)
            if bad:
                # look again once the picture has had RECHECK_S to settle (2026-10-24 19:39, Right alone: the camera
                # held a ghost 9 cm past the robot for 3 s after a 10.8 cm leg, "saw it move 24 cm, its wheels say
                # 11 cm", and the session waited an hour on a robot standing exactly where its wheels said)
                time.sleep(RECHECK_S)
                q2 = self.settle(t_from=time.time() - 0.6, min_s=0.3, timeout=1.5)
                if q2 is not None and not self.cross_check(robot, p0, q2):
                    return p0, q2, words + " (camera looked again: agrees with the wheels)"
                q = self.detect_after_move(robot)
                if q is not None:
                    # Nick, 2026-10-23: the camera lost it while it drove, but it has stopped where its wheels say:
                    # the detector finds it there and hands it back to the tracker (no turn on its spot needed)
                    words = "found again where its wheels put it (%s)" % bad
                    p1 = q
                else:
                    words = "camera and wheels disagree (%s)" % bad
                    p1 = self.refind(robot, bad)
        return p0, p1, words

    def wheels_moved(self, robot, lid):
        """How far the robot's own wheels and gyro say it moved during leg lid (its LEG records' poses from
        'started' to the final one), or None."""
        a = b = None
        with self.listener.cv:
            for t, n, rec in self.listener.recs["LEG"]:
                if n != robot or rec.get("id") != str(lid):
                    continue
                st_ = rec.get("state") or rec.get("result")
                xy = (num(rec.get("x_cm")), num(rec.get("y_cm")))
                if None in xy:
                    continue
                if st_ == "started":
                    a = xy
                elif st_ in ("done", "failed"):
                    b = xy
        if a is None or b is None:
            return None
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def cross_check(self, robot, p0, p1):
        """The camera against the wheels (Nick, 2026-10-21): after a go leg, did the camera see it move about as
        far as its wheels say? Returns plain words of the disagreement, or ''. Agreement resets the count."""
        w = self.wheels_moved(robot, self.last_lid.get(robot))
        if w is None:
            return ""
        if p1 is None:
            return "the camera lost it while its wheels say it moved %.0f cm" % w
        c = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        extra = ID_EXTRA_CM if self.fresh_id.pop(robot, False) else 0.0   # p0 came from a turn on the spot
        # a camera or wheels found noisy (e02_trust) widen the check by as much: their disagreement is that noise,
        # not a lost robot (2026-10-24 simulator, camera noise 4 cm: every other leg held the session)
        # (the wheels' k is against the robot's own baseline since 2026-10-25: it widens the check only while flagged)
        kk = max(1.0, self.trust.k("camera"),
                 self.trust.k("wheels", robot) if self.trust.bad("wheels", robot) else 1.0)
        if abs(c - w) > kk * max(CHECK_CM, CHECK_FRAC * w) + extra:
            return "the camera saw it move %.0f cm, its wheels say %.0f cm" % (c, w)
        self.disagree[robot] = 0
        return ""

    DETECT_NEAR_CM = 6.0      # chosen: a still robot outline this near where the wheels put it is the robot (DOE)

    def detect_after_move(self, robot):
        """Detection mode after a move the camera did not follow (Nick, 2026-10-23: "go back to detection mode after
        it moves to grab it again and hand it back to tracking mode"): the robot has stopped; its own pose (wheels
        and gyro since its last fix) says where. The detector's robot-shaped outlines within DETECT_NEAR_CM of that,
        and not where another robot stands, are it: the tracker follows it from there. Returns its position, or
        None (then the turn on its spot, as before)."""
        if self.tracker is None:
            return None
        own = self.own_pose(robot, since=time.time() - 30.0)
        if own is None:
            return None
        time.sleep(0.5)                                   # a few pictures of it standing still
        others = [tuple(q[:2]) for n, q in self.st["positions"].items() if n != robot and q]
        cands = [c for c in list(self.tracker.last_cands or []) if c.get("shape")
                 and math.hypot(c["x"] - own[0], c["y"] - own[1]) <= self.DETECT_NEAR_CM
                 and not any(math.hypot(c["x"] - o[0], c["y"] - o[1]) < CAM.MotionTracker.OTHER_CM for o in others)]
        if not cands:
            return None
        c = min(cands, key=lambda c: math.hypot(c["x"] - own[0], c["y"] - own[1]))
        self.st.setdefault("detect_after_move", []).append(
            [round(time.time(), 1), robot, round(c["x"], 1), round(c["y"], 1),
             round(math.hypot(c["x"] - own[0], c["y"] - own[1]), 1)])
        self.say("    %s: the camera lost it during the move; found it again where its wheels put it (%.1f, %.1f) cm"
                 % (robot, c["x"], c["y"]), quiet=True)
        self.tracker.follow(robot, c["x"], c["y"])
        self.history.clear()
        self.tracker.hold(HOLD_CM)
        return self.settle(min_s=0.3)

    def refind(self, robot, why):
        """The camera and the wheels disagree: stop, and find it again by a turn on its spot (nothing else turns
        just then). Returns its position, or None (then the caller gives up and, twice in a row, Nick is called)."""
        self.disagree[robot] += 1
        self.say("    %s: %s: finding it again by a short turn on its spot" % (robot, why))
        self.refinding.add(robot)
        try:
            return self._refind(robot, why)
        finally:
            self.refinding.discard(robot)

    def edge_risk(self, robot, p, what):
        """Test mode (Nick, 2026-10-23: "I don't mind if the robot falls and it might give us good data. This
        setting seems good for production runs but not test runs"): near an open edge with its heading unknown, turn
        on the spot anyway instead of calling Nick. Logged (edge_risks in the state, a video mark) so the footage
        shows whether a wheel went over. --edge-careful turns it off (production behaviour). Returns True to go on."""
        if getattr(self.a, "edge_careful", False):
            return False
        room = self.board.edge_room(p[0], p[1])[0] if p else None
        self.say("  %s is %s from an open edge and which way it faces is not known: turning anyway (test mode; "
                 "--edge-careful stops this)." % (robot, "%.0f cm" % room if room is not None else "near"))
        self.mark("e02 edge risk %s: %s" % (robot, what))
        self.st.setdefault("edge_risks", []).append([round(time.time(), 1), robot, what,
                                                     [round(p[0], 1), round(p[1], 1)] if p else None,
                                                     None if room is None else round(room, 1)])
        return True

    def _refind(self, robot, why):
        self.st.setdefault("disagreements", []).append([round(time.time(), 1), robot, why, self.order])
        # Nick, 2026-10-24: a robot the camera lost, or whose spot did not move with its wheels, stops everything
        # and the session waits for him (his answer "a" makes these find it again without asking)
        if self.hold_for_nick(robot, "lost", why) == "o":
            return None
        af = self.recently_auto_found(robot)
        if af is not None:
            self.fresh_id[robot] = True
            self.heads[robot] = [self.heads.get(robot, [None])[0], False]
            return self.settle(min_s=0.3) or af[1]           # found again on its own just now (auto_find)
        if self.moves.get(robot, {}).get("why") == "K":
            # 2026-10-24 review: in session 1, 4 of 9 camera-steered runs were ended here, because `stop` also ends
            # the firmware's camera-steered program. Between legs the robot already stands still, so no stop: the
            # turn on its spot that finds it again is just more legs of the same run.
            self.say("    %s: camera-steered run: finding it again without stopping the run" % robot, quiet=True)
        else:
            self.send(robot, "stop")
            if self.moves.get(robot):
                self.runner_stopped.add(robot)
        time.sleep(0.5)
        last = self.tracker.where()
        near = (last[1], last[2], 60.0) if last else None
        if last and self.board.edge_room(last[1], last[2])[0] < EDGE_TURN_CM:
            # a turn on the spot this near an open edge could put a wheel over it: back away first, if it is
            # known which way it faces; otherwise Nick
            h = self.heads.get(robot, [None])[0]
            if h is None:
                if not self.edge_risk(robot, (last[1], last[2]), "refind turn"):
                    return None
            else:
                self.leg(robot, ("go", -6.0))
        p = self.identify(robot, near=near) or self.identify(robot)
        if p is None:
            return None
        self.fresh_id[robot] = True
        self.tracker.robot_at(p[0], p[1], scale=ID_UNKNOWN_SCALE)
        self.tracker.follow(robot, p[0], p[1])
        self.history.clear()
        self.tracker.hold(HOLD_CM)
        self.heads[robot] = [self.heads.get(robot, [None])[0], False]
        return self.settle(min_s=0.3)

    def mark_leg(self, robot, which, timeout=8.0):
        state, info = self.send_leg(robot, "mark %s" % which, timeout + PAUSE_MS / 1000.0)
        return state

    def safe_path(self, robot, p, target, depth=0):
        """Waypoints from p to target that keep clear of every other robot the runner knows of (parked, or found
        and not parked yet: 2026-10-21 Center was steered straight into Right, not found yet). Detour points
        beside the robots in the way, on the side with room on the board; every piece checked; up to 2 detours.
        None if there is no clear way."""
        others = [tuple(q[:2]) for n, q in self.st["positions"].items() if n != robot and q]
        clear_cm = PL.PARK_CLEAR_CM + 4.0

        def first_block(a, b):
            dx, dy = b[0] - a[0], b[1] - a[1]
            L = math.hypot(dx, dy)
            if L < 1e-6:
                return None
            ux, uy = dx / L, dy / L
            worst = None
            for o in others:
                # a robot already near the start or the end: only a piece that comes nearer to it than that is
                # blocked (leaving a neighbour, or arriving at a spot beside one, is allowed)
                near = min(math.hypot(a[0] - o[0], a[1] - o[1]), math.hypot(b[0] - o[0], b[1] - o[1])) - 0.5
                lim = min(clear_cm, near)
                t = max(0.0, min(L, (o[0] - a[0]) * ux + (o[1] - a[1]) * uy))
                d = math.hypot(a[0] + ux * t - o[0], a[1] + uy * t - o[1])
                if d < lim and (worst is None or t < worst[0]):
                    worst = (t, o, ux, uy)
            return worst

        blk = first_block(p, target)
        if blk is None:
            return [target]
        if depth >= 2:
            return None
        _, o, ux, uy = blk
        best = None
        for side in (1.0, -1.0):
            for off in (30.0, 40.0):
                q = (o[0] - uy * side * off, o[1] + ux * side * off)
                if not self.board.on_board(q[0], q[1], margin=15.0):
                    continue
                if min([math.hypot(q[0] - r[0], q[1] - r[1]) for r in others] or [99.0]) < clear_cm:
                    continue
                if first_block(p, q) is not None:
                    continue
                rest = self.safe_path(robot, q, target, depth + 1)
                if rest is None:
                    continue
                cost = math.hypot(q[0] - p[0], q[1] - p[1]) + math.hypot(target[0] - q[0], target[1] - q[1])
                if best is None or cost < best[0]:
                    best = (cost, [q] + rest)
        return best[1] if best else None

    def drive_to(self, robot, target, final_heading=None, run=None, max_legs=40, arrive_cm=ST.ARRIVE_CM,
                 max_leg=ST.MAX_LEG_CM, avoid=True):
        """To target (x, y) by single legs, each driven on the robot's own wheels and gyro, the camera picking the
        next leg and checked against the wheels after each; then optionally turned to final_heading. avoid: go
        round the other robots the runner knows of. Returns (ok, words)."""
        h = self.heads.get(robot, [None, False])
        self.watch_count[robot] = 0
        self.__dict__.setdefault("near_zone", {}).pop(robot, None)      # item 5: a new drive, a new count
        p = self.settle(min_s=0.3, timeout=2.0)
        wps = self.safe_path(robot, p, target) if (avoid and p is not None) else [target]
        if wps is None:
            return False, "no clear way round the other robots"
        if len(wps) > 1:
            self.say("    %s: going round another robot (via %.0f, %.0f)" % (robot, wps[0][0], wps[0][1]), quiet=True)
        steer = ST.Steer(wps, heading=h[0], measured=h[1], final_heading=final_heading, max_legs=max_legs,
                         arrive_cm=arrive_cm, max_leg=max_leg)
        stuck = 0
        blocked = 0
        edge_stops = 0
        aways = 0
        after_turn = None            # fix 5: (time the last turn ended) until a straight drive has followed it
        self.front_bad.pop(robot, None)
        while True:
            p = self.settle(min_s=0.3, timeout=2.0)
            if p is None:
                p = self.refind(robot, "the camera can't see it")
                if p is None:
                    return False, "the camera can't see it, and a turn on its spot did not show it"
                steer.heading, steer.measured = None, False
            if not self.board.on_board(p[0], p[1]):
                return False, "the camera sees it off the board at (%.0f, %.0f) cm" % p
            if robot in self.held_out:
                return False, "it was left out (%s)" % ("its turns are not working" if robot in
                                                        getattr(self.turnwatch, "tripped", {}) else "by Nick")
            cmd = steer.next(p)
            near = Runner.near_target_leg(self, robot, steer, p, cmd)
            if near is not None:
                cmd = near
            if cmd[0] == "done":
                self.heads[robot] = [steer.heading, steer.measured]
                return True, ""
            if cmd[0] == "gave_up":
                self.heads[robot] = [steer.heading, steer.measured]
                return False, "it did not get there in %d legs" % max_legs
            if cmd[0] == "turn" and abs(cmd[1]) < ST.MIN_TURN_DEG:
                self.heads[robot] = [steer.heading, steer.measured]
                return True, ""
            room = self.signed_room(p)
            if cmd[0] == "probe" and steer.heading is None and room is not None and room[0] < HEADING_CRAWL_ROOM_CM:
                # fix 2: heading unknown near an open edge: the crawl, not a blind 4 cm probe
                q = self.heading_crawl(robot)
                steer.legs += 1
                if q is None:
                    self.heads[robot] = [None, False]
                    return False, "which way it faces is not known, and the crawl did not show it"
                steer.heading, steer.measured = self.heads[robot][0], True
                continue
            if cmd[0] == "turn" and room is not None and room[0] < EDGE_TURN_CM and steer.heading is not None \
                    and aways < 2:
                # fix 5: no turn on its spot within EDGE_TURN_CM of an open edge: straight away from it first
                mv = self.away_leg(steer.heading, room)
                if mv is not None:
                    aways += 1
                    self.say("    %s: %.0f cm from an open edge: moving %.1f cm straight away from it before it turns"
                             % (robot, room[0], abs(mv)), quiet=True)
                    self.st.setdefault("edge_aways", []).append([round(time.time(), 1), robot, round(room[0], 1),
                                                                 round(mv, 1)])
                    self.heads[robot] = [steer.heading, steer.measured]
                    q0, q1, res_a = self.leg(robot, ("go", mv), run=run)
                    if q0 is not None and q1 is not None and res_a.startswith(("done", "dup")):
                        steer.after(("go", mv), q0, q1)
                    else:
                        steer.legs += 1
                    continue
                self.st.setdefault("edge_turns_along", []).append([round(time.time(), 1), robot, round(room[0], 1)])
            if cmd[0] == "go" and cmd[1] > 0 and after_turn is not None:
                # fix 5: after every turn a fresh picture, taken after the turn ended, before the next straight drive
                lag = self.camera_lag_s() or 0.3
                q = self.settle(t_from=after_turn + lag, min_s=0.3, timeout=2.0 + lag)
                if q is None:
                    p = self.refind(robot, "no fresh picture of it after its turn")
                    if p is None:
                        return False, "no fresh picture of it after its turn, and a turn on its spot did not show it"
                    steer.heading, steer.measured = None, False
                    after_turn = None
                    continue
                p = q
                if room is not None and room[0] < TURN_CONFIRM_ROOM_CM and not steer.measured \
                        and cmd[1] > HEADING_CHECK_CM:
                    # near an open edge: the first drive after a turn is a short check the camera measures
                    cmd = ("go", HEADING_CHECK_CM)
                    self.st.setdefault("heading_checks", []).append([round(time.time(), 1), robot, round(room[0], 1)])
            if steer.heading is not None:
                self.heads[robot] = [steer.heading, steer.measured]     # the front-sensor check reads it
            self.front_bad.pop(robot, None)
            h_before = steer.heading
            p0, p1, res = self.leg(robot, cmd, run=run)
            if cmd[0] == "turn":
                after_turn = time.time()
                if not res.startswith(("done", "dup")):
                    # fix 3: a turn cut short (a hold, the watchdog), refused or stopped at the edge counts only for
                    # what it really turned by its own gyro; the plan then sends just the rest. (15:45: Left's cut
                    # 154 deg turn, 118 done, was sent again in full and it faced the bottom edge)
                    done = (getattr(self, "turn_done", None) or {}).get(robot)
                    steer.legs += 1
                    steer.heading = None if (done is None or h_before is None) else ST.wrap180(h_before + done)
                    steer.measured = False
                    self.heads[robot] = [steer.heading, False]
                    self.st.setdefault("turns_cut", []).append([round(time.time(), 1), robot, round(cmd[1], 1),
                                                                None if done is None else round(done, 1), res])
                    if res.startswith("refused") or res.startswith("could not"):
                        return False, "it refused the leg: " + res
                    if res.startswith("watchdog") and self.watch_stop and "lost" not in str(self.watch_stop):
                        self.watch_count[robot] += 1
                        if self.watch_count[robot] >= 2:
                            self.watch_count[robot] = 0
                            return False, "the camera watchdog stopped it twice: " + res
                    continue
            elif cmd[0] in ("go", "probe") and cmd[1] > 0:
                after_turn = None
            if cmd[0] in ("go", "probe") and cmd[1] > 0 and "plywood edge" in res:
                # fix 4: its edge guard stopped a forward drive: it faces an edge (the heading belief was wrong).
                # Straight back out (always allowed), measuring the heading on the way; the plan goes on from there
                edge_stops += 1
                self.say("    %s: its edge sensors stopped it (%s): backing straight out" % (robot, res))
                if edge_stops >= 2:
                    q0, q1, _ = self.edge_back_out(robot, HEADING_CRAWL_BACK_CM)
                    self.heads[robot] = [None, False]
                    return False, "its edge sensors stopped it twice: " + res
                q0, q1, _ = self.edge_back_out(robot, HEADING_CRAWL_BACK_CM)
                steer.legs += 1
                if q0 is not None and q1 is not None and math.hypot(q1[0] - q0[0], q1[1] - q0[1]) >= ST.MIN_HEADING_CM:
                    steer.heading, steer.measured = ST.wrap180(ST.bearing(q0, q1) + 180.0), True
                else:
                    steer.heading, steer.measured = None, False
                self.heads[robot] = [steer.heading, steer.measured]
                continue
            fb = self.front_bad.pop(robot, None)
            if fb and p1 is not None and not res.startswith("camera and wheels"):
                p1 = self.refind(robot, fb)
                if p1 is None:
                    return False, fb
                steer.heading, steer.measured = None, False
                steer.legs += 1
                continue
            if res.startswith("camera and wheels"):
                if p1 is None or self.disagree[robot] >= 2:
                    return False, res
                steer.heading, steer.measured = None, False   # found again by a turn: heading unknown
                steer.legs += 1
                continue
            if p0 is None or p1 is None:
                return False, "the camera lost it"
            if res.startswith("watchdog"):
                self.heads[robot] = [steer.heading, False]
                self.watch_count[robot] += 1
                if self.watch_count[robot] >= 2:
                    self.watch_count[robot] = 0
                    return False, "the camera watchdog stopped it twice: " + res
                continue
            if res.startswith("refused") or res.startswith("could not"):
                return False, "it refused the leg: " + res
            steer.after(cmd, p0, p1)
            if run is None and cmd[0] == "go" and steer.measured and "obstacle" not in res:
                self.heading_push(robot, steer.heading)      # heading fix 3
            if "obstacle" in res and cmd[0] in ("go", "probe") and cmd[1] > 0:
                # its front stop saw something (a board, another robot): the heading guess was wrong or the way
                # is blocked. Back off first (2026-10-22: turning on the spot against a board, then planning straight
                # back into it, looped 13 times), then a quarter turn towards the board's middle, and plan again.
                # Counted on its own: the turn legs in between used to reset the count, so it never gave up.
                blocked += 1
                self.say("    %s: something in front (%s): backing off and turning away (%d)" % (robot, res, blocked))
                if blocked >= 3:
                    self.heads[robot] = [steer.heading, steer.measured]
                    return False, "something keeps blocking its way"
                self.heads[robot] = [steer.heading, False]   # the heading guess may be what was wrong
                pb0, pb1, _ = self.leg(robot, ("go", -BACK_OFF_CM), run=run)
                if pb1 is not None:
                    steer.after(("go", -BACK_OFF_CM), pb0 if pb0 is not None else p1, pb1)
                    p1 = pb1
                ang = self.away_turn(p1, steer.heading)
                self.leg(robot, ("turn", ang))
                steer.after(("turn", ang), p1, p1)
                self.front_bad.pop(robot, None)
                continue
            if ST.leg_stuck(cmd, p0, p1):
                stuck += 1
                self.say("    %s hardly moved on '%s %.1f' (%s)" % (robot, cmd[0], cmd[1], res))
                if stuck >= 3:
                    self.heads[robot] = [steer.heading, steer.measured]
                    return False, "it is stuck (it did not move when told to, 3 times)"
                if stuck == 2:   # try backing out
                    self.leg(robot, ("go", -5.0), run=run)
            else:
                stuck = 0

    NEAR_TARGET_CM = HEADING_CHECK_CM + ST.ARRIVE_CM   # 5.5 cm: nearer than the check drive (+ arriving)
    NEAR_TURN_MAX_DEG = 30.0    # chosen: in that zone no turn bigger than this (a half turn there led to a 4 cm check
                                # drive past the target and back, 5 times: 21:14, 22:52-22:53 Center). No DOE: geometry
    NEAR_FLIPS_MAX = 2          # chosen: rolling back and forth this often in the zone ends the drive as arrived

    def near_target_leg(self, robot, steer, p, cmd):
        """Item 5 (Center's 5 half turns while parking): with the target nearer than the 4 cm check drive, a half
        turn and a check drive overshoot it every time. There: no turn over NEAR_TURN_MAX_DEG; roll along the heading
        (forward or back) instead, and a second change of direction ends it as arrived. Returns a replacement command,
        ('done', 0) or None (keep the plan's)."""
        if not steer.wp or (steer.waypoint is not None and steer.i < len(steer.wp) - 1) or \
                cmd[0] not in ("turn", "go", "probe"):
            return None
        tgt = steer.wp[-1]
        d = math.hypot(tgt[0] - p[0], tgt[1] - p[1])
        nz = self.__dict__.setdefault("near_zone", {})
        if d <= ST.ARRIVE_CM:
            return None                               # there: the final heading's turns are the plan's
        if d >= Runner.NEAR_TARGET_CM:
            nz.pop(robot, None)
            return None
        z = nz.setdefault(robot, dict(flips=0, last=0.0))
        if cmd[0] == "probe" or steer.heading is None:
            return None                               # heading unknown: the plan's probe measures it (once)
        err = ST.wrap180(ST.bearing(p, tgt) - steer.heading)
        along = d * math.cos(math.radians(err))
        if cmd[0] == "turn" and abs(cmd[1]) > Runner.NEAR_TURN_MAX_DEG:
            if abs(along) < ST.ARRIVE_CM or z["flips"] >= Runner.NEAR_FLIPS_MAX:
                self.st.setdefault("park_close_enough", []).append([round(time.time(), 1), robot, round(d, 1),
                                                                    "turn %.0f not made" % cmd[1]])
                self.say("    %s: %.1f cm from its target: close enough (no half turn for that)" % (robot, d),
                         quiet=True)
                return ("done", 0.0)
            cmd = ("go", round(along, 2))
        if cmd[0] == "go":
            if z["last"] and cmd[1] * z["last"] < 0:
                z["flips"] += 1
            if z["flips"] >= Runner.NEAR_FLIPS_MAX:
                self.st.setdefault("park_close_enough", []).append([round(time.time(), 1), robot, round(d, 1),
                                                                    "back and forth"])
                self.say("    %s: %.1f cm from its target after rolling back and forth: close enough" % (robot, d),
                         quiet=True)
                return ("done", 0.0)
            z["last"] = cmd[1]
            return cmd
        return None

    def signed_room(self, p):
        """(signed cm from p to the nearest open edge (- past it), (ux, uy) the inward direction from that edge), or
        None (no board, no position)."""
        bd = self.board
        if bd is None or p is None:
            return None
        best = None
        for a, v in bd.open_edges():
            d = ST.inward_cm(bd, a, v, p[0] if a == "x" else p[1])
            sgn = ST.inward_cm(bd, a, v, v + 1.0)                 # +1 if inside is the larger values
            n = (sgn, 0.0) if a == "x" else (0.0, sgn)
            if best is None or d < best[0]:
                best = (d, n)
        return best

    @staticmethod
    def away_leg(heading, room):
        """Fix 5: the straight drive (cm, + forward, - backward) that takes a robot facing `heading` to
        EDGE_TURN_CM + EDGE_AWAY_MARGIN_CM from the open edge of `room` (signed_room), at most EDGE_AWAY_MAX_CM; None
        when it faces along the edge (within EDGE_AWAY_MIN_COS of it: driving would barely move it away)."""
        d, n = room
        c = math.cos(math.radians(heading)) * n[0] + math.sin(math.radians(heading)) * n[1]
        if abs(c) < EDGE_AWAY_MIN_COS:
            return None
        need = EDGE_TURN_CM + EDGE_AWAY_MARGIN_CM - d
        if need <= 0:
            return None
        cm = min(EDGE_AWAY_MAX_CM, need / abs(c))
        return round(cm if c > 0 else -cm, 2)

    def away_turn(self, p, heading):
        """A quarter turn, to whichever side points more towards the board's middle (a fixed +90 could turn it
        along a board or towards an open edge). +90 when the heading or position is not known."""
        if p is None or heading is None:
            return 90.0
        b = self.board
        cx, cy = (b.x0 + b.x1) / 2.0, (b.y0 + b.y1) / 2.0
        want = math.degrees(math.atan2(cy - p[1], cx - p[0]))
        best = None
        for a in (90.0, -90.0):
            e = abs(ST.wrap180(want - (heading + a)))
            if best is None or e < best[0]:
                best = (e, a)
        return best[1]

    # --- finding a robot with the camera ------------------------------------------------------------
    def board_rect(self):
        if getattr(self, "exploring", False) or getattr(self, "whole_picture", False):
            return None                      # nothing is known about the board yet (or asked to): look anywhere
        b = self.board or PL.Board()
        return (b.x0, b.y0, b.x1, b.y1)

    def identify(self, robot, near=None, angle=ID_TURN_DEG, lights_only=False):
        """_identify, and a robot left out for failing turns (fix 1) that is found (by its lights: it takes no more
        turns) is kept on the map where it stands, so the others go round it. lights_only: no turn tries (auto_find)."""
        self.in_identify = True
        try:
            p = self._identify(robot, near=near, angle=angle, lights_only=lights_only)
        finally:
            self.in_identify = False
        if p is not None:
            # found and standing still: the tracker re-locks on it now, and that jump is not the robot moving (dry
            # run 19:27 at the bottom edge: the watchdog stopped Left on it, a second after its find)
            self.__dict__.setdefault("settle_until", {})[robot] = time.time() + FOUND_SETTLE_S
        if p is not None and robot in self.held_out:
            self.st["positions"][robot] = [p[0], p[1], None]
            if self.tracker is not None:
                try:
                    self.tracker.park(robot, p[0], p[1])
                except Exception:
                    pass
            self.st.setdefault("left_out", []).append([round(time.time(), 1), robot, [round(p[0], 1), round(p[1], 1)]])
        return p

    def _identify(self, robot, near=None, angle=ID_TURN_DEG, lights_only=False):
        """Where is `robot`? It turns a little on its spot and back (2026-10-21: a short DRIVE left its old spot
        behind, which the camera could take for the robot; a turn leaves nothing behind) and the camera looks for
        the one robot-sized change. Only this robot moves meanwhile. near = (x, y, radius cm) to look only there.
        Returns (x, y) or None. Its heading is not known after this (a body seen from above looks alike turned)."""
        self.mark("e02 identify %s" % robot)
        others = [tuple(q[:2]) for n, q in self.st["positions"].items() if n != robot and q]
        try:
            edge_why = Runner.at_edge_why(self, robot)      # (before the tracker lets go of it)
        except Exception:
            edge_why = ""
        if self.tracker is not None and self.tracker.active == robot:
            self.tracker.idle()
        if robot not in self.moves:          # inside a camera-steered run (K) its legs stay in that run
            self.moves[robot] = dict(run="-", why="identify")
        # a robot left out (its turns fail, fix 1) takes no turns: straight to its lights (a turn try first would leave
        # its lights on, and the lights' before-pictures lit: dry run 19:06, Right not found)
        if edge_why:
            # item 3: at or past an open edge, no turn on its spot to find it (21:45:36: Left's find-again turns
            # slid it further over the corner): its lights only
            self.say("    (%s: %s: found by its lights, no turn)" % (robot, edge_why), quiet=True)
            self.st.setdefault("identify_no_turn", []).append([round(time.time(), 1), robot, edge_why])
        for ang in (() if robot in self.held_out or edge_why or lights_only else (angle, -angle)):
            if robot in self.held_out:
                break                         # left out by its last turn (fix 1): its lights next
            before = self.feed.wait_frames(8, timeout=5)
            # lights on while it turns (2026-10-22, Nick): two changes in one place, the body's corners and the
            # lights; in a shadow, where the corners barely show, the lights carry it (firmware 2026-10-22a `led`)
            self.send(robot, "led 1 1 1 %d" % int((abs(ang) / TURN_DPS_GUESS * 2.5 + 4.0) * 1000))
            state, info = self.send_leg(robot, "turn %.1f" % ang, abs(ang) / TURN_DPS_GUESS * 2.5 + 6.0)
            if robot in self.held_out:
                break                         # left out by that turn (fix 1): no turn back, its lights next
            time.sleep(0.6)
            after = self.feed.wait_frames(8, timeout=5)
            self.send_leg(robot, "turn %.1f" % -ang, abs(ang) / TURN_DPS_GUESS * 2.5 + 6.0)
            time.sleep(0.4)
            if state not in ("done", "dup") or len(before) < 3 or len(after) < 3:
                self.say("    (%s did not turn: %s %s)" % (robot, state, info), quiet=True)
                continue
            cands = CAM.locate_change(self.cam, before, after, board_rect=self.board_rect(), near=near)
            cands = [c for c in cands if all(math.hypot(c["x"] - o[0], c["y"] - o[1]) > 6.0 for o in others)]
            self.st.setdefault("identify", []).append([round(time.time(), 1), robot, ang,
                                                       [[round(c["x"], 1), round(c["y"], 1), c["area"],
                                                         round(c["strength"], 1)] for c in cands[:4]]])
            if not cands:
                continue
            if len(cands) > 1 and cands[1]["area"] * cands[1]["strength"] > 0.5 * cands[0]["area"] * cands[0]["strength"]:
                self.say("    (two places changed when %s turned: looking again)" % robot, quiet=True)
                continue
            return (cands[0]["x"], cands[0]["y"])
        # third try: its lights, steady (firmware 2026-10-22a `led`): pictures before and while they are on; the one
        # spot that got brighter. Works at the ~2 pictures a second the laptop gets (the 2 Hz flash below does not).
        if self.feed is not None and hasattr(self.feed, "collect"):
            if robot in self.held_out:
                self.send(robot, "led 0 0 0 100")     # (left out during a turn try: its turn lights may still be on)
                time.sleep(0.6)
            before, _ = self.feed.collect(1.2)
            self.send(robot, "led 1 1 1 2500")
            time.sleep(0.5)
            during, _ = self.feed.collect(1.6)
            if len(before) >= 1 and len(during) >= 1:
                f = CAM.locate_light(self.cam, before, during, board_rect=self.board_rect(), near=near)
                self.st.setdefault("identify", []).append([round(time.time(), 1), robot, "light",
                                                           [round(f["x"], 1), round(f["y"], 1), round(f["strength"])]
                                                           if f else None, len(before), len(during)])
                if f and all(math.hypot(f["x"] - o[0], f["y"] - o[1]) > 6.0 for o in others):
                    self.say("    (found %s by its lights)" % robot)
                    return (f["x"], f["y"])
        # fourth try: the 2 Hz flash (`identify`), for a fast picture stream
        if self.feed is not None and hasattr(self.feed, "collect"):
            self.send(robot, "identify")
            imgs, ts = self.feed.collect(3.6)
            f = CAM.locate_flash(self.cam, imgs, ts, board_rect=self.board_rect(), near=near)
            self.st.setdefault("identify", []).append([round(time.time(), 1), robot, "flash",
                                                       [round(f["x"], 1), round(f["y"], 1), f["strength"]] if f else None])
            if f and all(math.hypot(f["x"] - o[0], f["y"] - o[1]) > 6.0 for o in others):
                self.say("    (found %s by its lights)" % robot)
                return (f["x"], f["y"])
        return None

    def acquire(self, robot, step_cm=10.0, near=None):
        """Find robot in the picture: identify it by a turn on its spot, then a short drive (forward; blocked:
        a quarter turn and again) gives its heading. Returns (x, y) or None."""
        p = self.identify(robot, near=near)
        self.moves.pop(robot, None)
        if p is None:
            return None
        self.st["positions"][robot] = [p[0], p[1], None]
        return self.acquire_heading(robot, p)

    def activate(self, robot):
        """Follow `robot` from where it was last parked (the tracker keeps every parked robot visible)."""
        self.active = robot
        sp = self.st["positions"].get(robot)
        p = self.tracker.where()
        if self.tracker.active == robot and p and (sp is None or math.hypot(p[1] - sp[0], p[2] - sp[1]) < 5):
            return True
        if sp is None:
            return False
        self.tracker.follow(robot, sp[0], sp[1])
        self.history.clear()
        self.tracker.hold(HOLD_CM)
        if robot not in self.heads:
            self.heads[robot] = [sp[2], False]
        time.sleep(0.3)
        return self.settle(min_s=0.3) is not None

    # --- parking ----------------------------------------------------------------------------------
    def park_index(self, robot):
        return self.st["robots"].index(robot)

    def park(self, robot, run=None):
        """Camera-steered to its park spot: to the staging point, turn to face the x-board, then back into the
        spot (the backwards leg measures the heading it ends with)."""
        i = self.park_index(robot)
        px, py, ph = self.board.park(i)
        sx, sy = self.board.stage(i)
        p = self.settle(min_s=0.3)
        if p is None:
            return False, "the camera can't see it"
        self.phase = "parking"
        esc = self.board.escape_point(p[0], p[1])
        if esc is not None:
            # near an open edge (a route that ended off course): first straight in from it, then on as usual
            self.say("    %s is %.0f cm from an open edge: moving it in first" % (robot, self.board.edge_room(*p)[0]))
            ok, why = self.drive_to(robot, esc, arrive_cm=4.0, max_leg=PARK_LEG_CM)
            if not ok:
                return ok, why
        if math.hypot(p[0] - px, p[1] - py) > 4.0:
            if math.hypot(p[0] - sx, p[1] - sy) > 3.0:
                ok, why = self.drive_to(robot, (sx, sy), arrive_cm=2.0, max_leg=PARK_LEG_CM)
                if not ok:
                    return ok, why
            h = self.heads.get(robot, [None, False])
            if h[0] is not None:
                e = ST.wrap180(ph - h[0])
                if abs(e) > ST.MIN_TURN_DEG:
                    self.leg(robot, ("turn", e))
                self.heads[robot] = [ph, False]
            p = self.settle(min_s=0.3)
            if p is None:
                return False, "the camera lost it"
            ux, uy = math.cos(math.radians(ph)), math.sin(math.radians(ph))
            along = (px - p[0]) * ux + (py - p[1]) * uy           # negative: the spot is behind it
            if abs(along) > 2.0:
                p0, p1, _ = self.leg(robot, ("go", along))
                if p0 and p1:
                    st_ = ST.Steer([], heading=self.heads[robot][0])
                    st_.after(("go", along), p0, p1)
                    self.heads[robot] = [st_.heading, st_.measured]
                    if ST.leg_stuck(("go", along), p0, p1):
                        return False, "it did not move back into its spot"
        ok, why = self.drive_to(robot, (px, py), final_heading=ph, arrive_cm=3.0, avoid=False)
        self.end_moves(robot)
        if ok:
            p = self.settle(min_s=0.3)
            self.st["positions"][robot] = [p[0], p[1], self.heads.get(robot, [None])[0]]
            self.tracker.park(robot, p[0], p[1])
            self.st.save()
        return ok, why

    def back_out(self, robot, within_s=60.0):
        """Reverse the robot's last forward drive (that way was clear a moment ago): a camera fix if the camera sees
        it, then `go -d` (at most BACK_MAX_CM: backing is blind). True if it backed."""
        last = None
        with self.listener.cv:
            for t, nm, rec in reversed(self.listener.recs["LEG"]):
                if nm != robot or t < time.time() - within_s:
                    continue
                if (rec.get("state") or rec.get("result")) in ("done", "failed") and rec.get("kind") == "go":
                    v = num(rec.get("value"))
                    if v is not None and v > 0:
                        last = v
                        break
        if last is None:
            return False
        d = min(last + 2.0, BACK_MAX_CM)
        self.say("  %s is at an open edge: backing out the way it came (%.0f cm) ..." % (robot, d))
        if not self.begin_moves(robot, "backing out"):
            return False
        self.fix_now(robot)
        state, info = self.send_leg(robot, "go %.1f" % -d, d / DRIVE_CMS_GUESS * 2.5 + 6.0)
        self.moves.pop(robot, None)
        return state in ("done", "dup")

    def retry_find(self, robot, park=True, placed=False):
        """One try at getting a robot back with nobody's hands: a fresh picture of the floor, a turn on its spot,
        which way it faces, then parking. Returns (ok, words)."""
        last = self.tracker.where() if (self.tracker is not None and self.tracker.active == robot) else None
        q = (last[1], last[2]) if last else tuple((self.st["positions"].get(robot) or [None, None])[:2])
        if not placed and q[0] is not None and (not self.board.on_board(q[0], q[1]) or
                                 self.board.edge_room(q[0], q[1])[0] < EDGE_TURN_CM + 3):
            # review 2026-10-22: a turn on the spot this near an open edge could put a wheel over it. Nick (evening):
            # it just drove there and the camera sees it: back out the way it came first
            if not self.back_out(robot) and not (self.board.on_board(q[0], q[1]) and
                                                 self.edge_risk(robot, q, "retry turn")):
                return False, "atedge"
        time.sleep(1.0)
        old_pos = self.st["positions"].pop(robot, None)
        self.disagree[robot] = 0
        self.fresh_floor()
        p = self.identify(robot)
        self.moves.pop(robot, None)
        if p is None:
            if old_pos:
                self.st["positions"][robot] = old_pos
            return False, "noturn"
        self.st["positions"][robot] = [p[0], p[1], None]
        self.say("  Found %s at (%.1f, %.1f) cm." % (robot, p[0], p[1]))
        if self.tracker is not None:
            self.tracker.hand_moved.discard(robot)
        self.lost_place.discard(robot)
        if not park:
            return True, ""
        if not self.board.on_board(p[0], p[1]):
            # (near an edge on the board the heading is found by the crawl now, fix 2: no blind drive, so no test-mode
            # risk is needed for it)
            # 2026-10-22 10:23: Center found 3.5 cm off the board, then its heading drive and "moving it in first"
            # took it over the edge (it fell): its edge guard had learned the plywood level with a sensor already
            # over the edge. Its heading is unknown here, so no drive at all: Nick (firmware 22b now refuses a
            # forward drive that starts off the plywood). Test mode drives anyway unless its centre is off the board.
            return False, "atedge"
        rm_ = self.signed_room(p)
        if rm_ is not None and rm_[0] < CRAWL_MIN_ROOM_CM:
            # its front sensors may already be over the drop: the edge guard would learn the drop as floor (the
            # 2026-10-22 fall); no crawl, Nick
            return False, "atedge"
        if self.acquire_heading(robot, p) is None:
            return False, "noheading"
        self.set_corrections(robot, self.move_corrections(robot))
        ok, why = self.park(robot)
        return ok, why

    def sweep_hits(self, finder, q, h, lost):
        """The finder turns once round on its spot at q (heading h) in SWEEP_STEP_DEG steps, reading its front
        sensor after each (firmware 2026-10-22b reports it with every turn leg). A reading well short of what the
        map says is ahead (boards, robots we know of; not the lost one) is something not on the map. Returns
        ([(reading, x, y)], heading after the turn) ."""
        hits = []
        heading = h
        if not self.begin_moves(finder, "looking for " + lost):
            return hits, heading
        for k in range(int(round(360.0 / SWEEP_STEP_DEG))):
            state, info = self.send_leg(finder, "turn %.1f" % SWEEP_STEP_DEG, SWEEP_STEP_DEG / TURN_DPS_GUESS * 2.5 + 6.0)
            heading = ST.wrap180(heading + SWEEP_STEP_DEG)
            m = re.search(r"front=([0-9.]+)", info or "")
            if state not in ("done", "dup") or not m:
                continue
            front = float(m.group(1))
            if front <= 0:
                continue                      # no reading (nothing in range)
            saved = self.st["positions"].pop(lost, None)
            ahead = self.ahead_cm(finder, q[:2], heading)
            if saved is not None:
                self.st["positions"][lost] = saved
            if front < SWEEP_RANGE_CM and (ahead is None or front < ahead[0] - SWEEP_GAP_CM):
                d = front + 2 * PL.ROBOT_HALF_CM
                x, y = q[0] + d * math.cos(math.radians(heading)), q[1] + d * math.sin(math.radians(heading))
                if self.board.on_board(x, y):
                    hits.append((front, x, y))
        return sorted(hits), heading

    def sweep_for(self, lost):
        """Nick, 2026-10-22: a robot that knows where it is looks for the lost one with its front sensor, from where
        it is parked (sweep_hits); the camera then looks for the lost robot's turn there only. Returns (x, y) or
        None."""
        for finder in self.st["robots"]:
            if finder == lost or finder in self.unavailable:
                continue
            q = self.st["positions"].get(finder)
            h = self.heads.get(finder, [None])[0]
            if h is None and q and len(q) > 2:
                h = q[2]
            if not q or h is None or self.board.edge_room(q[0], q[1])[0] < EDGE_TURN_CM + 3:
                continue
            self.say("  %s looks round for %s with its front sensor ..." % (finder, lost))
            self.mark("e02 sweep %s for %s" % (finder, lost))
            hits, heading = self.sweep_hits(finder, q, h, lost)
            self.heads[finder] = [heading, self.heads.get(finder, [None, False])[1]]
            self.moves.pop(finder, None)
            for _, x, y in hits:
                self.say("  %s sees something not on the map at about (%.0f, %.0f) cm: is it %s?" % (finder, x, y, lost))
                p = self.identify(lost, near=(x, y, SWEEP_NEAR_CM))
                if p is not None:
                    return p
        return None

    def lookouts(self, searcher, lost):
        """Lookout points for a search, nearest first from the searcher: SEARCH_LOOKOUTS (cm in from the two
        boards, so every one is well away from the open edges), skipping any too near a robot we know of."""
        p = self.st["positions"].get(searcher)
        pts = [self.board.uv(u, v) for u, v in SEARCH_LOOKOUTS]
        others = [tuple(q[:2]) for n, q in self.st["positions"].items() if n not in (searcher, lost) and q]
        pts = [q for q in pts if all(math.hypot(q[0] - o[0], q[1] - o[1]) >= SEARCH_CLEAR_CM for o in others)]
        out, cur = [], (p[0], p[1]) if p else self.board.find_stage()
        while pts:
            nxt = min(pts, key=lambda q: math.hypot(q[0] - cur[0], q[1] - cur[1]))
            out.append(nxt)
            pts.remove(nxt)
            cur = nxt
        return out

    def search_for(self, lost):
        """Nick, 2026-10-22: nobody knows where the lost robot is (the camera can't see it and its own find failed).
        Another robot searches for it: first its own find against the two boards (its position exact before it
        starts), then to each lookout point (camera-steered: it is in view; its front-sensor checks against the
        boards re-confirm its position on the way), sweeping its front sensor once round at each. A reading short
        of the map, or a stop for something in front where the map is clear, is a hit: the camera looks for the
        lost robot's turn there; if the camera can't see that spot either, the hit itself is the best place known.
        The searcher parks again after. Returns (x, y) or None (nothing found: then the session stops)."""
        # its last place is not where it is: the searcher's paths must not go round it there (dry run 17:11: every
        # lookout and the searcher's own parking found "no clear way" round that stale place)
        stale = self.st["positions"].pop(lost, None)
        try:
            return self._search(lost)
        finally:
            if stale is not None and lost not in self.st["positions"]:
                self.st["positions"][lost] = stale

    def _search(self, lost):
        for searcher in [n for n in self.st["robots"] if n != lost and n not in self.unavailable]:
            if not self.st["positions"].get(searcher) or not self.activate(searcher):
                continue
            self.say("  %s searches the board for %s ..." % (searcher, lost))
            self.mark("e02 search %s for %s" % (searcher, lost))
            self.set_corrections(searcher, self.move_corrections(searcher))
            for c in self.board_params():
                self.send(searcher, c)
                time.sleep(0.05)
            out = self.do_find(searcher, "search")
            if not out.get("ok"):
                self.say("  %s: %s" % (searcher, out.get("words")))
                self.park(searcher)
                continue
            best = None
            for lp in self.lookouts(searcher, lost):
                ok, why = self.drive_to(searcher, lp, max_leg=PARK_LEG_CM, arrive_cm=6.0)
                p = self.settle(min_s=0.3)
                h = self.heads.get(searcher, [None])[0]
                if not ok and "blocking" in why and p is not None and h is not None:
                    # stopped by something in front where the map is clear: a hit just ahead of it
                    d = SEARCH_STOP_AHEAD_CM + 2 * PL.ROBOT_HALF_CM
                    hits = [(SEARCH_STOP_AHEAD_CM, p[0] + d * math.cos(math.radians(h)),
                             p[1] + d * math.sin(math.radians(h)))]
                elif ok and p is not None and h is not None:
                    hits, heading = self.sweep_hits(searcher, p, h, lost)
                    self.heads[searcher] = [heading, self.heads.get(searcher, [None, False])[1]]
                    self.moves.pop(searcher, None)
                else:
                    continue
                for _, x, y in hits:
                    self.say("  %s: something not on the map at about (%.0f, %.0f) cm: is it %s?" % (
                        searcher, x, y, lost))
                    found = self.identify(lost, near=(x, y, SWEEP_NEAR_CM))
                    if found is not None:
                        self.park(searcher)
                        return found
                    best = best or (x, y)
                if best is not None:
                    break                      # something is there; the camera can't see it: take it as the place
            ok, why = self.park(searcher)
            if not ok:
                self.help_from_nick(searcher, why)
            if best is not None:
                self.say("  The camera can't see %s, but %s's sensor found something at about (%.0f, %.0f) cm: "
                         "taking that as its place." % (lost, searcher, best[0], best[1]))
                return best
            return None                        # one full search is enough: the board has been looked over
        return None

    def rescue_find(self, robot, need_camera=True):
        """Nick, 2026-10-22: a robot the camera can't find finds itself, against the two boards. Its own pose (the
        last camera fix plus the legs since, wheels and gyro) is good enough to drive to the find's staging point;
        the find's own checks keep it safe (the front stop; a bump much sooner than the pose says fails it; the
        edge guard). No camera fix is sent (the camera can't see it). After the find its position is exact: the
        camera then looks for its turn there. Only when its last known place was well in from the open edges and
        nothing says it was moved by hand. Returns (x, y) or None."""
        q = self.st["positions"].get(robot)
        if not q or self.board.edge_room(q[0], q[1])[0] < RESCUE_EDGE_CM:
            return None
        if self.tracker is not None and robot in self.tracker.hand_moved:
            return None                    # lifted or pushed while held: its own pose is wrong
        self.say("  %s finds itself against the boards (the camera can't see it) ..." % robot)
        for c in self.board_params():
            self.send(robot, c)
            time.sleep(0.05)
        if self.tracker is not None and self.tracker.active == robot:
            self.tracker.idle()            # do_find must not send a fix from a lost track
        for attempt in range(2):
            out = self.do_find(robot, "rescue")
            if out.get("ok"):
                self.lost_place.discard(robot)
                break
            self.lost_place.add(robot)       # it drove somewhere the camera did not see: its place is unknown now
            self.say("  %s: %s" % (robot, out.get("words")))
            if "stop" in (out.get("words") or ""):
                return None                # stopped (watchdog or a stop): don't push on blind
            # (a find that failed squaring up, or found no board in reach, is worth one more try: dry run 13:40)
        else:
            return None
        fr = self.listener.latest("FIND", robot, out["t0"])
        ax, ay = (num(fr[1].get("after_x")), num(fr[1].get("after_y"))) if fr else (None, None)
        if ax is None or ay is None:
            return None
        self.say("  %s found itself at (%.1f, %.1f) cm; the camera looks there ..." % (robot, ax, ay))
        self.st["positions"][robot] = [ax, ay, None]      # where it is, even if the camera can't see it there
        self.fresh_floor()
        p = self.identify(robot, near=(ax, ay, SWEEP_NEAR_CM))
        if p is None and not need_camera:
            return (ax, ay)                # its find's end pose is exact: good enough to drive it out from
        return p

    def robot_picture(self, robot, tag):
        """The latest camera picture with the robot circled where it was last known (the tracker, else its parked or
        last place) and named: <session>/question_<tag>_<robot>.png, for a question on the live page. None if there
        is no picture."""
        try:
            import cv2
            img = getattr(getattr(self, "feed", None), "last_img", None)
            if img is None:
                return None
            out = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img.ndim == 2 else img.copy()
            q = None
            if self.tracker is not None and self.tracker.active == robot:
                w = self.tracker.where()
                q = (w[1], w[2]) if w else None
            if q is None and (self.st.get("positions") or {}).get(robot):
                q = tuple(self.st["positions"][robot][:2])
            if q is not None and getattr(self, "cam", None) is not None:
                u, v = self.cam.project((q[0], q[1], EX.ROBOT_TOP_CM))
                u, v = int(round(float(u))), int(round(float(v)))
                cv2.circle(out, (u, v), 28, (0, 200, 255), 3)
                cv2.putText(out, "%s (last known)" % robot, (u + 32, v), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
            else:
                cv2.putText(out, "%s: not known where" % robot, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                            (0, 200, 255), 2)
            path = os.path.join(self.dir, "question_%s_%s.png" % (tag, robot))
            cv2.imwrite(path, out)
            return path
        except Exception as e:
            self.say("  (no picture for the question: %s)" % e, quiet=True)
            return None

    def lights_whole_picture(self, robot):
        """Its lights (steady, then the 2 Hz flash), the camera searching the WHOLE picture, not only the board's
        outline (10:50: a robot the find had missed was on the board all along). (x, y) or None. Logged."""
        if getattr(self, "feed", None) is None or getattr(self, "cam", None) is None:
            return None
        self.say("  %s: looking for its lights once more, over the whole picture ..." % robot)
        self.whole_picture = True
        try:
            q = self.identify(robot, lights_only=True)
        except Exception as e:
            self.say("    (lights: %s)" % e, quiet=True)
            q = None
        finally:
            self.whole_picture = False
        self.st.setdefault("lights_retries", []).append([round(time.time(), 1), robot,
                                                         None if q is None else [round(q[0], 1), round(q[1], 1)]])
        return q

    def found_by_lights_then_park(self, robot, q, park):
        """Found by its lights at q: on the map, the tracker on it; its heading (the crawl near an edge), then parked
        if park. True if done."""
        if self.board is not None and not self.board.on_board(q[0], q[1]):
            return False
        self.st["positions"][robot] = [q[0], q[1], None]
        self.say("  Found %s by its lights at (%.1f, %.1f) cm." % (robot, q[0], q[1]))
        if not park:
            return True
        try:
            if self.acquire_heading(robot, q) is None:
                return False
            self.set_corrections(robot, self.move_corrections(robot))
            ok, _ = self.park(robot)
            return ok
        except Exception as e:
            self.say("  (%s: found, but not parked: %s)" % (robot, e), quiet=True)
            return False

    def help_from_nick(self, robot, problem, park=True):
        """Stuck, lost or off the board: first AUTO_TRIES tries on its own (2026-10-22: pressing Enter with no
        hands, i.e. just finding it again, worked nearly every time); then exact steps for Nick, then the runner
        finds it again by a turn on its spot and parks it itself. Returns True when it is back (and parked, unless
        park=False: then only found)."""
        if robot in self.held_out:
            return False                   # Nick left it out at a hold: it stays where it is
        if park and "off the board" not in problem:
            # (park=False: the caller goes on from where the robot stands, e.g. jammed on a brick: Nick moves it)
            for k in range(AUTO_TRIES):
                self.say("  %s: %s. Trying again on its own (%d of %d) ..." % (robot, problem, k + 1, AUTO_TRIES))
                self.mark("e02 auto-retry %s %d: %s" % (robot, k + 1, problem))
                self.st.setdefault("auto_retries", []).append([round(time.time(), 1), robot, problem])
                self.send(robot, "stop")
                ok, why = self.retry_find(robot, park)
                if ok:
                    self.say("  %s is back without help." % robot)
                    return True
                if why == "atedge":
                    problem = "it is at (or over) an open edge, and which way it faces is not known"
                    break
                if why == "noturn" and k == AUTO_TRIES - 1 and self.tracker is not None:
                    self.heads[robot] = [None, False]        # a rescue find sets it (exact); a sweep does not
                    p = self.sweep_for(robot) or self.rescue_find(robot)
                    if p is not None:
                        self.st["positions"][robot] = [p[0], p[1], None]
                        self.say("  Found %s again at (%.1f, %.1f) cm." % (robot, p[0], p[1]))
                        if not park:
                            return True
                        h = self.heads.get(robot, [None, False])
                        if h[0] is not None and h[1]:
                            # just found itself against the boards: its heading is exact, no drive needed
                            self.tracker.robot_at(p[0], p[1], scale=ID_UNKNOWN_SCALE)
                            self.tracker.follow(robot, p[0], p[1])
                            self.history.clear()
                            self.tracker.hold(HOLD_CM)
                            headed = True
                        else:
                            headed = self.acquire_heading(robot, p) is not None    # near an edge: the crawl
                        if headed:
                            self.set_corrections(robot, self.move_corrections(robot))
                            ok, why = self.park(robot)
                            if ok:
                                return True
                        # found but not parked (dry run 13:15: lost during its first turn in the corner): it is
                        # in view now, so one more ordinary try before Nick
                        self.say("  %s: one more try now that it is in view ..." % robot)
                        ok, why2 = self.retry_find(robot, park)
                        if ok:
                            self.say("  %s is back without help." % robot)
                            return True
                if why not in ("noturn", "noheading"):
                    problem = why
        calls = [h for h in self.st.get("help_calls", []) if h[1] == robot and h[0] >= self.st.get("t_session_start", 0)]
        if self.in_runs and len(calls) >= HELP_DROP_N and \
                len([n for n in self.st["robots"] if n not in self.unavailable and n != robot]) >= 1:
            # backup: this robot keeps needing help; put it aside (it stays on the map where it was last seen, the
            # others go round it) and carry on with the others; --resume later runs what it missed
            self.mark("e02 drop %s: %s" % (robot, problem))
            self.st.setdefault("dropped", []).append([round(time.time(), 1), robot, problem])
            self.unavailable.add(robot)
            self.say("  %s has needed help %d times this session (now: %s). It is put aside and the others carry on; "
                     "you may take it off the board whenever you like (--resume runs its remaining runs later)."
                     % (robot, len(calls), problem))
            self.send(robot, "stop")
            try:
                self.keep_on_map(robot)
                self.st.save()
                return False
            except SystemExit:
                # nobody knows where it is, so the others could drive into it: Nick after all
                self.unavailable.discard(robot)
                self.say("  ... but nobody can find it, so the others could drive into it: help is needed after all.")
        # coordinator, 2026-10-26 10:50: before asking Nick, its lights once more with the camera searching the
        # whole picture (not only the board's outline)
        q = self.lights_whole_picture(robot)
        if q is not None and self.found_by_lights_then_park(robot, q, park):
            return True
        where = ("anywhere on the open part of the board, at least 30 cm (1 ft) from the other robots, the boards "
                 "and the edges")
        facing = "open board"
        self.mark("e02 help %s: %s" % (robot, problem))
        self.st.setdefault("help_calls", []).append([round(time.time(), 1), robot, problem])
        self.st.save()
        print("\n" + "!" * 78)
        steps = list(ST.nick_instructions(robot, problem, where, facing)) + [
            "   (You don't need to put it on any particular spot: it finds its own way there.)"]
        for line in steps:
            self.say(line)
        print("!" * 78)
        # coordinator, 2026-10-26 10:50: the live page's question shows all of it (robot, problem, where, what to do)
        # and a picture with the robot circled where it was last known, not only "press Enter"
        self.help_text = "\n".join(l.strip() for l in steps if l.strip())
        self.help_image = self.robot_picture(robot, "help")
        self.send(robot, "stop")
        self.send(robot, "identify")
        if self.a.auto:
            pass
        wait_s = self.a.help_wait_min * 60.0
        self.say("(If nobody answers within %.0f minutes, %s is put aside and the others carry on; --resume later "
                 "runs what it missed.)" % (self.a.help_wait_min, robot))
        # 15:45: two help calls went unanswered for 10 minutes: the live page sounds now, and again every
        # ALERT_REPEAT_S while this waits
        self.alert("help", robot, problem)
        self.help_waiting = robot
        try:
            return self._help_wait(robot, park, wait_s)
        finally:
            self.help_waiting = None

    def _help_wait(self, robot, park, wait_s):
        while True:
            print("\a", end="", flush=True)
            # coordinator, 2026-10-26 10:50: the question names its robot in its first words (the live page showed
            # "Press Enter when it is back on the board" with three robots on the board)
            ans = self.ask_timeout("%s: press Enter when %s is back on the board (f = flash %s again, skip = put %s "
                                   "aside).\n\n%s" % (robot, robot, robot, robot, getattr(self, "help_text", "") or ""),
                                   wait_s, image=getattr(self, "help_image", None))
            if ans is None or ans.lower() == "skip":
                self.mark("e02 help %s %s" % ("timeout" if ans is None else "skip", robot))
                self.unavailable.add(robot)
                if ans is None:
                    self.say("Nobody answered in %.0f minutes: %s's remaining runs are put aside." % (
                        self.a.help_wait_min, robot))
                    self.keep_on_map(robot)
                else:                          # Nick took it off the board
                    self.st["positions"].pop(robot, None)
                    if self.tracker is not None:
                        self.tracker.forget(robot)
                    self.st.save()
                return False
            if ans.lower() == "f":
                self.send(robot, "identify")
                continue
            self.say("Taking a fresh picture of the board (keep hands out of it), then %s turns a little on its "
                     "spot ..." % robot)
            # (placed: Nick has just put it down, so where it was last seen says nothing; evening 20:12, it kept
            # saying "at an open edge" from its old place after Nick had moved it)
            ok, why = self.retry_find(robot, park, placed=True)
            if ok:
                return True
            if why == "noturn":
                # 10:50: not seen: its lights again, the camera searching the whole picture, before Nick is asked again
                q = self.lights_whole_picture(robot)
                if q is not None and self.found_by_lights_then_park(robot, q, park):
                    return True
                self.say("The camera did not see %s turn, nor its lights anywhere in the picture. Check it is on the "
                         "board and switched on (its side light on), then press Enter again." % robot)
            elif why == "atedge":
                self.say("The camera sees it at (or over) an open edge. Move it well in from the edge, then press Enter.")
            elif why == "noheading":
                self.say("It turned, but the camera lost it when it drove. Press Enter to try again.")
            else:
                self.say("It moved, but parking failed: %s. Put it somewhere open again and press Enter." % why)

    # --- moving a robot the camera can't see, on its own wheels and gyro (Nick, 2026-10-22) -------------------
    def own_pose(self, robot, since=0.0):
        """The robot's own position (x, y, heading) from its last finished leg: wheels and gyro since the last
        camera fix or find, in the board's frame. None if it has not reported one."""
        last_find = self.listener.latest("FIND", robot)
        if last_find and (last_find[1].get("result") or "") != "ok" and \
                self.fix_t.get(robot, 0.0) < last_find[0]:
            # its last find failed and no camera fix since: every pose it reports after it is untrusted too (dry
            # run 16:52: a turn on its spot after the failed find gave a pose at the board's line)
            return None
        r = self.listener.latest("LEG", robot, since)
        f = self.listener.latest("FIND", robot, since)
        if f and (not r or f[0] >= r[0]):
            # a find after its last leg: a finished find's end pose is exact; a failed one's is not to be trusted
            # (it may have started from no pose at all; review 2026-10-22: a dry run kept Left at (99, 162))
            if (f[1].get("result") or "") != "ok":
                return None
            x, y, th = num(f[1].get("after_x")), num(f[1].get("after_y")), num(f[1].get("after_th"))
        elif r:
            x, y, th = num(r[1].get("x_cm")), num(r[1].get("y_cm")), num(r[1].get("th_deg"))
        else:
            return None
        if None in (x, y, th) or not self.board.on_board(x, y, margin=-5.0):
            return None                     # off the board: not a pose it can have (unknown pose, power cycle)
        return (x, y, th)

    def own_path(self, robot, until, n=BLIND_RETRACE_LEGS):
        """Where the robot's own pose was at the end of each of its last n finished drives before `until` (the start
        of the blind drive: not the blind pieces themselves) and after its last find (newest last)."""
        out = []
        f = self.listener.latest("FIND", robot)
        after = f[0] if f else 0.0
        with self.listener.cv:
            for t, nm, rec in self.listener.recs["LEG"]:
                st_ = rec.get("state") or rec.get("result")
                if nm == robot and st_ == "done" and rec.get("kind") == "go" and after <= t < until:
                    x, y = num(rec.get("x_cm")), num(rec.get("y_cm"))
                    if x is not None and y is not None:
                        out.append((x, y))
        return out[-n:]

    def drive_blind(self, robot, waypoints):
        """Drive through waypoints on the robot's own pose: turn to face each, then short drives
        (BLIND_PIECE_CM), the front stop and the edge guard on. After every drive the front sensor is checked
        against what the map says should be ahead (boards, robots we know of): landmarks confirm the pose. Returns
        (ok, words)."""
        if not self.begin_moves(robot, "driving blind"):
            return False, "could not start its moves"
        for wp in waypoints:
            if not self.board.on_board(wp[0], wp[1]) or self.board.edge_room(wp[0], wp[1])[0] < BLIND_EDGE_CM:
                return False, "the way out runs near an open edge"
            reached = False
            for _ in range(20):
                pose = self.own_pose(robot)
                if pose is None:
                    return False, "it has not said where it thinks it is"
                d = math.hypot(wp[0] - pose[0], wp[1] - pose[1])
                if d < 3.0:
                    reached = True
                    break
                turn = ST.wrap180(math.degrees(math.atan2(wp[1] - pose[1], wp[0] - pose[0])) - pose[2])
                if abs(turn) > ST.MIN_TURN_DEG:
                    state, info = self.send_leg(robot, "turn %.1f" % turn, abs(turn) / TURN_DPS_GUESS * 2.5 + 6.0)
                    if state not in ("done", "dup"):
                        return False, "a turn did not finish (%s %s)" % (state, info)
                go = min(d, BLIND_PIECE_CM)
                state, info = self.send_leg(robot, "go %.1f" % go, go / DRIVE_CMS_GUESS * 2.5 + 6.0)
                if state not in ("done", "dup"):
                    return False, "a drive did not finish (%s %s)" % (state, info)
                # landmarks: what is ahead by its own pose against what its front sensor reads
                pose = self.own_pose(robot)
                m = re.search(r"front=([0-9.]+)", info or "")
                if pose and m and float(m.group(1)) > 0:
                    ahead = self.ahead_cm(robot, pose[:2], pose[2])
                    off, tol = self.front_limits(robot)
                    if ahead and ahead[0] <= FRONT_CHECK_RANGE_CM and self.square_on(ahead) and \
                            abs(float(m.group(1)) - off - ahead[0]) > FRONT_STEER_FACTOR * tol:
                        return False, "its front sensor reads %.0f cm where its own pose puts %s at %.0f cm" % (
                            float(m.group(1)), ahead[1], ahead[0])
            if not reached:
                return False, "it did not reach (%.0f, %.0f) cm" % tuple(wp)
        return True, ""

    def drive_out(self, robot):
        """A robot the camera can't see, or can't park, stuck where it blocks the others (the find's corner): drive
        it to its park spot on its own pose, round the robots we know of; else retrace its last drives backwards
        (that way was clear a moment ago); then the camera looks for it where it should be. Returns (x, y) or
        None."""
        pose = self.own_pose(robot)
        if pose is None:
            return None
        if self.tracker is not None and robot in self.tracker.hand_moved:
            return None                    # moved by hand: its own pose is wrong
        self.say("  %s drives out on its own wheels and gyro (the camera can't follow it there) ..." % robot)
        self.mark("e02 drive out %s" % robot)
        t_start = time.time()
        try:
            px, py, _ = self.board.park(self.park_index(robot))     # its own park spot: out of every route's way
            # first straight away from both boards (dry run 14:51: driving along the board from the corner, its
            # front stop saw the board and stopped it), then round the robots to its spot
            out = self.board.uv(BLIND_CLEAR_CM, BLIND_CLEAR_CM)
            first = [out] if (self.board.edge_room(pose[0], pose[1])[1] < BLIND_CLEAR_CM and
                              math.hypot(out[0] - pose[0], out[1] - pose[1]) > 3.0) else []
            rest = self.safe_path(robot, first[0] if first else pose[:2], (px, py))
            wps = first + rest if rest is not None else None
            ok, words = (self.drive_blind(robot, wps) if wps is not None else
                         (False, "no clear way round the other robots"))     # never straight through them
            if not ok:
                self.say("  %s: %s; retracing its last drives instead ..." % (robot, words))
                back = list(reversed(self.own_path(robot, until=t_start)))
                pose2 = self.own_pose(robot)
                back = [q for q in back if pose2 is None or math.hypot(q[0] - pose2[0], q[1] - pose2[1]) > 3.0]
                if not back:
                    return None
                ok, words = self.drive_blind(robot, back)
                if not ok:
                    self.say("  %s: %s" % (robot, words))
                    return None
            pose = self.own_pose(robot)
            self.fresh_floor()
            p = self.identify(robot, near=(pose[0], pose[1], SWEEP_NEAR_CM)) if pose else None
        finally:
            self.moves.pop(robot, None)
        if p is not None:
            self.say("  %s is in view again at (%.1f, %.1f) cm." % (robot, p[0], p[1]))
            return p
        if pose is not None:
            # it drove there without trouble, but the camera can't see it there either: go by its own pose (the
            # others are planned round it; better than leaving it where every find drives)
            self.say("  The camera can't see %s at (%.0f, %.0f) cm, where it drove; going by its own position." % (
                robot, pose[0], pose[1]))
            return (pose[0], pose[1])
        return None

    def keep_on_map(self, robot):
        """A robot put aside stays on the board as an obstacle the others go round: try once more to find where it
        is (a turn on its spot, no Nick); if that fails, keep the last place it was seen. During setup nobody knows
        where it is at all: the session stops rather than drive the others blind (review 2026-10-22)."""
        last = self.tracker.where() if (self.tracker and self.tracker.active == robot) else None
        q = self.st["positions"].get(robot)
        guess = (last[1], last[2]) if last else (tuple(q[:2]) if q else None)
        p = self.identify(robot, near=(guess[0], guess[1], 60.0) if guess else None)
        if p is None:
            # 2026-10-26 09:55 session: decisions about a put-aside robot use where it is NOW (11:05 used a place 20
            # minutes old): its lights over the whole picture once more
            p = self.lights_whole_picture(robot)
        seen_now = p is not None
        self.moves.pop(robot, None)
        if p is None:
            # the camera can't see it: its own pose from its last leg, if that is newer than the camera's last
            # sighting (dry run 14:31: it had found itself in the find corner, the runner still had its park spot)
            own = None
            if self.tracker is None or robot not in self.tracker.hand_moved:
                own = self.own_pose(robot, since=(last[0] if last else 0.0))
            p = (own[0], own[1]) if own else (None if robot in self.lost_place else guess)
        if p is None:
            # (also: it moved unseen after its last known place, e.g. a failed find of its own: dry run 15:15, the
            # runner kept it at its park spot while it stood in Center's way). Nick: search for it first.
            p = self.search_for(robot)
            if p is not None:
                self.lost_place.discard(robot)
        if p is None:
            raise SystemExit("%s is somewhere on the board but the camera can't find it, so the other robots could "
                             "drive into it. Take %s off the board (or put it anywhere open), then start again." % (
                                 robot, robot))
        self.st["positions"][robot] = [p[0], p[1], None]
        if self.tracker is not None:
            if self.tracker.active == robot:
                self.tracker.idle()
            self.tracker.park(robot, p[0], p[1])
        self.say("  %s stays where it is, at about (%.0f, %.0f) cm; the others go round it." % (robot, p[0], p[1]))
        self.st.save()
        fx, fy = self.board.find_stage()
        kx, ky, _ = self.board.park(self.park_index(robot))
        in_corner = math.hypot(p[0] - fx, p[1] - fy) < BLOCKS_FIND_CM
        h_ = self.heads.get(robot, [None, False])
        drivable = seen_now and h_[0] is not None and bool(h_[1])
        if not drivable and (in_corner or math.hypot(p[0] - kx, p[1] - ky) > ASIDE_PARK_CM):
            # 2026-10-26 09:55 session (11:05): a put-aside robot whose position and heading are not both known now
            # is NOT driven (no drive-out on its own pose, no find: Left drove into the right edge that way). It stays
            # put until Nick moves it; the others go round it, and find against a free corner if it blocks the usual
            self.say("  %s is not driven (%s): it stays where it is until you move it; the others go round it." % (
                robot, "its heading is not known" if seen_now else "the camera cannot see it now"))
            self.st.setdefault("aside_not_driven", []).append([round(time.time(), 1), robot, [round(p[0], 1),
                                                                round(p[1], 1)], seen_now, bool(h_[1])])
            self.alert("help", robot, "%s is put aside at (%.0f, %.0f) cm and not driven: move it off the board when "
                       "you can" % (robot, p[0], p[1]))
            if in_corner and not self.free_find_corner(p):
                raise SystemExit("%s is put aside at about (%.0f, %.0f) cm, in the corner every robot's find drives "
                                 "to, and no other staging point is clear, so no run can start. Take %s off the board, "
                                 "then start again with --resume." % (robot, p[0], p[1], robot))
            self.st.save()
            return
        if not in_corner and math.hypot(p[0] - kx, p[1] - ky) > ASIDE_PARK_CM:
            # put aside out on the board (dry run 15:01: at (73, 62), where the routes run; the next run's watchdog
            # stopped it): drive it to its own spot on its own pose; if that fails it stays where it is
            self.say("  %s is put aside out on the board, where the routes run: moving it to its spot ..." % robot)
            q = self.drive_out(robot)
            if q is not None:
                self.st["positions"][robot] = [q[0], q[1], None]
                if self.tracker is not None:
                    self.tracker.robot_at(q[0], q[1])
                    self.tracker.park(robot, q[0], q[1])
                self.say("  %s stays at about (%.0f, %.0f) cm; the others go round it." % (robot, q[0], q[1]))
                self.st.save()
            return
        if in_corner:
            # dry run 13:15: a robot put aside by the blocked corner, where every find drives: every run after it
            # failed its find. Nick (2026-10-22): drive it out blind first (its own pose, landmarks, retracing),
            # then the finder sweep and its own find; stop only if all of that fails.
            self.say("  %s is in the corner every find drives to: moving it out ..." % robot)
            q2 = None
            for step in (None, self.sweep_for, lambda r: self.rescue_find(r, need_camera=False)):
                # each step that only finds it (the sweep) or puts it in the corner (its own find, which leaves an
                # exact pose) is followed by a drive out (review 2026-10-22)
                if step is not None:
                    found = step(robot)
                    if found is None:
                        continue
                    if math.hypot(found[0] - fx, found[1] - fy) >= BLOCKS_FIND_CM:
                        q2 = found
                        break
                    if step == self.sweep_for:      # (bound methods: equal, never identical)
                        # its position is known now (heading not): tell it, then drive out on its gyro's heading
                        self.send(robot, "fix %.2f %.2f - %.3f" % (found[0], found[1], time.time()))
                        self.fix_t[robot] = time.time()
                        time.sleep(0.2)
                q = self.drive_out(robot)
                if q is not None and math.hypot(q[0] - fx, q[1] - fy) >= BLOCKS_FIND_CM:
                    q2 = q
                    break
            if q2 is not None:
                self.st["positions"][robot] = [q2[0], q2[1], None]
                if self.tracker is not None:
                    self.tracker.robot_at(q2[0], q2[1])
                    self.tracker.park(robot, q2[0], q2[1])
                self.say("  %s stays at about (%.0f, %.0f) cm, out of the way; the others go round it." % (
                    robot, q2[0], q2[1]))
                self.st.save()
                return
            pl = self.st["positions"].get(robot) or p          # where it is now, not where the help call saw it
            if self.free_find_corner(pl):
                self.st.save()
                return
            raise SystemExit("%s is put aside at about (%.0f, %.0f) cm, in the corner every robot's find drives to, "
                             "so no run can start. Take %s off the board, then start again with --resume." % (
                                 robot, pl[0], pl[1], robot))

    FIND_STAGE_FREE_CM = 50.0     # 2026-10-26 09:55 session: with a set-aside robot in the usual find corner, finds start
                                  # this far from both board faces (instead of 25), so both bumps land 50 cm from the
                                  # corner. Chosen: under the firmware's 60 cm find reach (e02_find_max_cm) with the
                                  # back face's 6 cm; DOE note: no, geometry (a dry run checks a find from it)

    def free_find_corner(self, p):
        """A set-aside robot at p blocks the usual find staging point: move every later find's staging point to
        FIND_STAGE_FREE_CM from both faces (sent with every find's board settings: e02_find_stage_cm) if p is clear of
        it. True if done (the session carries on)."""
        b = self.board
        nx, ny = b.uv(Runner.FIND_STAGE_FREE_CM, Runner.FIND_STAGE_FREE_CM)
        if math.hypot(p[0] - nx, p[1] - ny) < BLOCKS_FIND_CM:
            return False
        b.find_stage_cm = Runner.FIND_STAGE_FREE_CM
        self.st["find_stage_cm"] = Runner.FIND_STAGE_FREE_CM
        self.say("  Finds now start %.0f cm from both boards (not %.0f), clear of the robot in the corner." % (
            Runner.FIND_STAGE_FREE_CM, PL.FIND_STAGE_CM))
        self.mark("e02 find stage moved")
        return True

    # --- checks -----------------------------------------------------------------------------------
    def wait_robots(self, names, timeout=40):
        """Wait for the robots to be heard. A robot that last took commands from another computer (the laptop, when
        the session runs on the Pi) sends its lines only there and no longer says hello: a `status` to everyone on
        the WiFi (broadcast) makes this computer the one it answers (firmware net.h: a robot answers whoever last
        sent it a command)."""
        t0 = time.time()
        last_call = 0.0
        while time.time() - t0 < timeout and not set(names) <= set(self.link.robots()):
            if time.time() - last_call > 2.0:
                last_call = time.time()
                try:
                    self.link.send_ip("255.255.255.255", "status")
                except OSError:
                    pass
            time.sleep(0.5)
        return [n for n in names if n not in self.link.robots()]

    def start_battery_watch(self, names):
        """battery.csv (Nick, 2026-10-24): every robot heard on the WiFi, every --battery-watch-s seconds, and every
        battery the robots report with their runs. Columns: role (driving: in this session; idle: switched on but not
        in it), state (moving: a leg or a firmware run going, else still), still_s (seconds since it last moved: the
        battery % comes from its voltage, which sags while the motors draw and recovers after, so the fair readings
        are the rested ones, still_s of 60 or more), phase, source (status: asked; run: a run's own record), moved_s
        (2026-10-25: seconds it was moving since its previous row, counted every 2 s: the battery learner's driving
        time when the robot logs are not fetched yet; a resumed session whose file has no such column goes without).
        Robots sent to sleep are logged "asleep". A robot is asked `status` only while it is still (`status` only
        answers; it never moves a robot)."""
        if not hasattr(self, "asleep_names"):
            self.asleep_names = set()           # robots sent to sleep (robots_ready): their rows say "asleep"
        every = getattr(self.a, "battery_watch_s", 0) or 0
        if every <= 0 or getattr(self, "_bat_thread", None) is not None:
            return
        path = os.path.join(self.dir, "battery.csv")
        new = not os.path.exists(path)
        with_moved = new
        if not new:
            try:
                with open(path) as f:
                    with_moved = "moved_s" in f.readline()
            except OSError:
                with_moved = False
        drive = set(names)
        t_start = time.time()
        self.st.setdefault("charge_levels", {})

        def busy(n):
            return n in self.leg_live or (self.moves.get(n) or {}).get("run", "-") not in ("-", None)

        def loop():
            if new:
                with open(path, "w") as f:
                    f.write("t,robot,role,state,still_s,battery,phase,source,moved_s\n")
            box = dict(logged={}, last_moved={}, asked=set(), last_ask=0.0, moved={}, t_tick=None)

            def tick():
                logged, last_moved = box["logged"], box["last_moved"]
                now = time.time()
                dt_tick = min(10.0, now - box["t_tick"]) if box["t_tick"] is not None else 0.0
                box["t_tick"] = now
                for n in set(self.link.robots()) | drive:
                    if busy(n):
                        last_moved[n] = now
                        box["moved"][n] = box["moved"].get(n, 0.0) + dt_tick
                if now - box["last_ask"] >= every:
                    box["last_ask"] = now
                    box["asked"] = set()
                    for n in self.link.robots():
                        if not busy(n) and not self.hold.is_set():
                            try:
                                self.link.send_name(n, "status")
                                box["asked"].add(n)
                            except OSError:
                                pass
                rows = []
                for n, (tb, b) in sorted(dict(self.listener.battery).items()):
                    if tb > logged.get(n, 0.0):
                        logged[n] = tb
                        mv = busy(n)
                        still = 0.0 if mv else max(0.0, tb - last_moved.get(n, t_start))
                        rows.append("%.1f,%s,%s,%s,%.0f,%d,%s,%s" % (
                            tb, n, "driving" if n in drive else "idle",
                            "moving" if mv else ("asleep" if n in self.asleep_names else "still"), still, b,
                            getattr(self, "phase", "") or "",
                            "status" if n in box["asked"] and tb - box["last_ask"] < 5.0 else "run")
                            + (",%.0f" % box["moved"].pop(n, 0.0) if with_moved else ""))
                if rows:
                    with open(path, "a") as f:
                        f.write("\n".join(rows) + "\n")

            last_levels = 0.0
            while not getattr(self, "_bat_stop", False):       # _bat_stop: only the tests set it
                try:
                    tick()
                except Exception as e:          # the battery log must never die quietly (2026-10-24 review)
                    self.say("  (battery log: %s)" % e, quiet=True)
                if time.time() - last_levels >= min(every, LEVELS_EVERY_S):
                    last_levels = time.time()
                    try:
                        self.charge_levels_tick(names)
                    except Exception as e:      # the levels are advice: never stop the battery log
                        self.say("  (charge levels: %s)" % e, quiet=True)
                time.sleep(2.0)

        self._bat_thread = threading.Thread(target=loop, daemon=True)
        self._bat_thread.start()

    def charge_levels_tick(self, names):
        """Each robot's go / leave levels from the battery learned so far (Nick, 2026-10-25 13:03: the levels adapt as
        data comes in): tools/dock/charge_levels.LiveLevels refits the drains from battery.csv and the log, saves them
        into logs/battery_learned.json (dry runs: battery_learned-dryrun.json), and gives the levels; they go into
        e02_state.json "charge_levels" (the live view's robot cards) and the session log when they move a point."""
        if getattr(self, "_live_levels", None) is None:
            dock = os.path.join(TOOLS, "dock")
            if dock not in sys.path:
                sys.path.insert(0, dock)
            import charge_levels as CLV
            hist = os.path.join(REPO, "logs", "battery_learned%s.json" % (
                "-dryrun" if getattr(self.a, "dry_run", False) else ""))
            self._live_levels = CLV.LiveLevels(self.dir, hist)
        working = [n for n in names if n not in self.unavailable]
        out, words = self._live_levels.tick(working, resting=sorted(getattr(self, "asleep_names", ())))
        self.st["charge_levels"] = out
        self.st.save()
        for w in words:
            self.say("  battery: " + w, quiet=" drains " not in w)
        return out

    def batteries(self, names):
        t = time.time()
        for n in names:
            self.send(n, "status")
        time.sleep(1.5)
        out = {}
        for n in names:
            b = self.listener.battery.get(n)
            out[n] = b[1] if b and b[0] >= t - 0.5 else None
        return out

    def setup(self):
        a, st = self.a, self.st
        setup = st["setup"]
        self.say("=== Setup checks ===")
        self.say("Virtual border: %s session, the camera's %.0f cm and the robots' own %.0f cm inside the open edges "
                 "(a crossing stops every robot and asks)." % ("home" if getattr(self.a, "home", False) else "away",
                                                               self.border_cm(), self.fw_border_cm()))
        # 1. Pi recorder (production: no recorder; the tracker process's own pictures)
        ok1, w1, size, fps = TKW.check_overhead(self, a)
        if a.no_side:
            ok2, w2 = True, "not used this session (--no-side)"
        else:
            ok2, w2, _, _ = CAM.check_stream(a.pi, a.side_port)
        notes = True if getattr(a, "no_recording", False) else CAM.notes_answer(a.pi)
        self.say("Overhead camera (%s:%d): %s" % (a.pi, a.overhead_port, "OK, " + w1 if ok1 else "PROBLEM: " + w1))
        self.say("Side camera (%s:%d): %s" % (a.pi, a.side_port, ("OK, " if not a.no_side else "") + w2 if ok2
                                                 else "PROBLEM: " + w2))
        self.say("Pi recorder hears notes: %s" % ("not used (--no-recording: live picture only, no video)"
                                                   if getattr(a, "no_recording", False) else
                                                   "OK" if notes else "PROBLEM: no answer on port 5007"))
        setup["pi"] = dict(overhead=ok1, side=ok2, notes=notes, fps=round(fps, 1))
        if not ok1:
            raise SystemExit("The overhead camera is not sending pictures. On the Pi (SSH window), start the "
                             "recorder with both cameras:\n  python3 tools/pi/camrec.py --device 0,2 --names "
                             "overhead,side --fps 25 --label E02\nthen start this again.")
        if not ok2 or not notes:
            if self.ask("The side camera or the notes are not answering. Carry on anyway? (y/n) ", "y").lower() != "y":
                raise SystemExit("Stopped. Start the Pi recorder with both cameras, then start this again.")
        # 2. calibration
        frames = self.feed.wait_frames(10, timeout=10)
        if not frames:
            raise SystemExit("No pictures from the overhead camera.")
        img = frames[-1]
        if not getattr(a, "classic_setup", False) and not st.get("calibration"):
            # Nick, 2026-10-23: from scratch. No calibration file, no outline to confirm, no board questions: the
            # robots calibrate the camera and map the board themselves (explore, after they are connected)
            self.say("From scratch: no camera calibration and no board are known; the robots will work them out "
                     "(explore) once they are connected.")
            self.cam = EX_provisional_camera(img.shape[1], img.shape[0])
            self.exploring = True
            self.make_tracker()
            setup["explore_pending"] = True
            st.save()
            return
        self.classic_calibration(img)

    def classic_calibration(self, img):
        """The camera calibration from a file (the studio's, the corner clicks, or one made here from the board's
        corners), confirmed by Nick's look at the green outline; then the blocked boards from the picture. Used with
        --classic-setup, and as explore's fallback (explore_fallback)."""
        a, st = self.a, self.st
        setup = st["setup"]
        cal = st.get("calibration")
        if not cal or not os.path.exists(cal):
            cal = None
            if not a.make_calibration:
                cal = CAM.find_calibration(a.calib, a.project, self.dir)
                if cal is None and os.path.exists(DEFAULT_CALIB):
                    self.say("No floor calibration in the studio project: using the one made from roam1's board corners "
                             "(%s; the overhead camera has not moved since)." % DEFAULT_CALIB)
                    cal = DEFAULT_CALIB
            if cal and not self.confirm_calibration(cal, img):
                self.say("The board outline does not line up with this calibration (%s)." % cal)
                # Nick, 2026-10-22 evening: a "no" is not a stop. 2026-10-25: nor the studio's steps (they assume
                # a laptop and loop): the stored calibration, said plainly (outline_doubted)
                cal = self.outline_doubted(cal, img)
        if cal:
            self.say("Floor calibration: %s" % cal)
        else:
            self.say("No floor calibration found (looked in the studio project %s\\cameras and this session)."
                     % (a.project or CAM.default_project()))
            corners, why = None, ""
            if self.ask("Make one now from the board's 4 corners in the camera picture? (y/n) ", "y").lower() == "y":
                corners, why = CAM.find_board_corners(img)
                if corners is None:
                    self.say("Could not find the board's corners by myself (%s): the studio, then." % why)
        if not cal and corners is None:
            cal = self.studio_calibration(img)
        elif not cal:
            cal = os.path.join(self.dir, "cameras", "overhead.json")
            res = CAM.make_calibration(corners, (img.shape[1], img.shape[0]), cal, height_cm=a.lens_height_cm)
            cam = CAM.load_camera(cal)
            pic = CAM.draw_check(img, cam, os.path.join(self.dir, "check_calibration.png"),
                                 PL.Board(), corners)
            self.say("Made %s: %s" % (cal, res["text"]))
            self.say("Open %s: the green outline must sit on the board's edges, 0,0 in the bottom-right corner." % pic)
            if self.ask("Does the green outline sit on the board's edges? (y/n) ", "y", image=pic).lower() != "y":
                os.remove(cal)
                cal = self.outline_doubted(None, img)
            else:
                setup["calibration_made_here"] = dict(corners=corners, err_cm=res["err_cm"], text=res["text"])
        st["calibration"] = cal
        if not (st["setup"].get("calibration_source") or {}).get("kind"):
            st["setup"]["calibration_source"] = dict(kind="file", file=cal, session=False,
                                                     words="%s, not made by the robots this session" % os.path.basename(cal))
        self.cam = CAM.load_camera(cal)
        self.make_tracker()
        st.save()
        # 3. the blocked boards
        if not st.get("board") and getattr(a, "no_boards", False):
            xs, ys = (int(v) for v in (a.board_sides or "1,1").split(","))
            b = PL.Board(xboard_side=xs, yboard_side=ys)
            b.missing = {"x", "y"}
            st["board"] = b.to_dict()
            self.say("No boards (--no-boards): all four sides are open edges; every find feels for two edges.")
        elif not st.get("board"):
            self.find_boards(img)
        self.board = PL.Board.from_dict(st["board"])
        if getattr(a, "no_boards", False):
            self.board.missing = {"x", "y"}
        if st.get("find_stage_cm"):
            self.board.find_stage_cm = float(st["find_stage_cm"])    # a free find corner chosen earlier (--resume)
        self.tracker_area()
        sx, sy, sh = self.board.nominal_start()
        self.say("Route start spot: (%.1f, %.1f) cm, near the blocked corner" % (sx, sy))
        for r in PL.ROUTES:
            ok, open_room, board_room = self.board.route_fits(r)
            self.say("Route %s, heading %.0f deg: %s (front at least %.0f cm from the open edges, body %.1f cm from "
                     "the boards)" % (PL.ROUTE_NAMES[r], self.board.route_heading(r), "fits" if ok else "TOO TIGHT",
                                      open_room, board_room))
        if self.board.turn_sign < 0:
            self.say("WARNING: with the boards on these two edges the square route (the firmware turns it left) would "
                     "head into the x-board. The square runs are left out of this session's plan.")
            st["setup"]["square_left_out"] = True
        st.save()

    def outline_doubted(self, cal, img):
        """Nick said the green outline is off. The stored calibration is used anyway, said plainly here and on every
        run's line (calib_note); the fix is new corner clicks before the next session. The studio's steps are only
        for when no calibration is stored at all (2026-10-25 15:26, on the Pi: they assume a laptop and loop)."""
        stored = cal if cal and os.path.exists(cal) else (DEFAULT_CALIB if os.path.exists(DEFAULT_CALIB) else None)
        if stored is None:
            return self.studio_calibration(img)
        self.say("")
        self.say("!!! You said the green outline is off. Carrying on with the stored camera calibration anyway "
                 "(%s): there is no other one here." % os.path.basename(stored))
        self.say("!!! Positions near the board's edges may be a few cm off. Every run's line and the session summary "
                 "say so.")
        self.say("    Before the next session: make a new floor calibration from the board's 4 corners (about 5 min).")
        self.st["setup"]["calibration_doubted"] = dict(file=stored, t=round(time.time(), 1))
        self.mark("e02 outline judged off: stored calibration used")
        return stored

    def studio_calibration(self, img):
        """The outline did not line up: Nick makes a floor calibration in the studio while the runner waits, then
        it is checked the same way (green outline, y/n). Repeats until a yes; 'stop' ends the session."""
        proj = self.a.project or CAM.default_project()
        while True:
            t_ask = time.time()
            self.say("Make a new floor calibration in the studio, leaving this window open:")
            self.say("  1. Open the MIRLT studio on this laptop (the project %s)." % proj)
            self.say("  2. More -> Floor calibration -> rectangle.")
            self.say("  3. Click the plywood board's 4 corners (122.15 x 121.9 cm) and save.")
            ans = self.ask("  4. Come back here and press Enter (or type stop and press Enter to end): ", "")
            if ans.lower().startswith("stop"):
                raise SystemExit("Stopped at the floor calibration (Nick). Start this again when it is made.")
            cands = [os.path.join(proj, "cameras", n + ".json") for n in CAM.CAMERA_NAMES]
            cands = [c for c in cands if os.path.exists(c)]
            cal = max(cands, key=os.path.getmtime) if cands else None
            if cal is None or os.path.getmtime(cal) < t_ask - 1.0:
                self.say("No new floor calibration saved in %s\\cameras yet (%s)." % (
                    proj, "nothing there" if cal is None else "the newest is from before"))
                continue
            if self.confirm_calibration(cal, img):
                self.say("Using the studio's new floor calibration: %s" % cal)
                return cal
            self.say("That one does not line up either.")

    def confirm_calibration(self, cal, img):
        """The plain check: the board's outline (from the calibration) drawn over the live picture, on screen;
        Nick says whether it lines up (y/n). Unattended (--auto) or no screen: the picture is saved and taken as
        right. Returns True to use it."""
        cam = CAM.load_camera(cal)
        path = os.path.join(self.dir, "check_calibration.png")
        CAM.draw_check(img, cam, path, PL.Board())
        self.st["setup"]["calibration_check"] = path
        if self.a.auto:
            self.say("Calibration check picture: %s (automatic mode: taken as lining up)" % path)
            return True
        shown = CAM.show_picture(path, "E02: does the green outline sit on the board's edges?")
        self.say("%s the camera picture with the board's outline in green from the floor calibration%s." % (
            "On screen now:" if shown else "Open", "" if shown else " (" + path + ")"))
        ans = self.ask("Does the green outline sit on the plywood board's 4 edges (within about a finger's width)? "
                       "Type y or n and press Enter: ", "y", image=path).lower()
        CAM.close_picture()
        ok = ans.startswith("y")
        self.st["setup"]["calibration_confirmed"] = dict(file=cal, answer=ans)
        return ok

    def tracker_area(self):
        """The tracker's board area and walls (the board faces, robot-centre side: a centre is half a body in from
        the face), from the board found in the picture."""
        if self.tracker is None or self.board is None:
            return
        b = self.board
        gone = getattr(b, "missing", ())
        walls = [w for k, w in (("x", ("y", b.xboard_cm + b.uy * PL.ROBOT_HALF_CM)),
                                ("y", ("x", b.yboard_cm + b.ux * PL.ROBOT_HALF_CM))) if k not in gone]
        self.tracker.set_area((b.x0, b.y0, b.x1, b.y1), walls=walls)

    def keep_tracker_stats(self, tr):
        """A tracker about to be replaced (explore's end, a recalibration): add its counts to the session's, so the
        end summary covers the whole session, not only the last tracker (2026-10-26 09:55 session: "none 98 %" was the
        last 17,000 pictures only)."""
        if tr is None or not getattr(tr, "stats", None):
            return
        acc = self.__dict__.setdefault("tracker_stats_before", {})
        for k, v in dict(tr.stats).items():
            if isinstance(v, (int, float)):
                acc[k] = acc.get(k, 0) + v

    def make_tracker(self):
        if self.tracker is None:
            b = PL.Board()
            centre = (0.0, 0.0) if getattr(self, "exploring", False) else ((b.x0 + b.x1) / 2, (b.y0 + b.y1) / 2)
            no_lock = bool(getattr(self.a, "no_lock", False))
            # in the tracker process when the feed is its (e02_trackproc, 2026-10-25), else in the runner as before
            self.tracker = TKW.make_tracker(self, centre, use_lock=not no_lock)
            if no_lock:
                self.say("Tracker: the motion detector alone (--no-lock).")

    BOARD_WORDS_MIN = 2           # chosen: a picture side needs this many of explore's board contacts (and more than its
                                  # open-edge ones) to count as a board side. No DOE: explore's own side rule (rect_check)
                                  # asks 2 a side too

    def board_words_from_contacts(self):
        """The two picture sides explore met the boards along, from its contacts in pixels (setup explore_live
        contacts: [robot, u, v, kind]): works even when explore's fit failed afterwards (10:45: the outline was drawn
        from them, then the fallback asked). {'x': side, 'y': side} (which is which does not matter here) or None."""
        live = ((self.st.get("setup") or {}).get("explore_live") or {}).get("contacts") or []
        img = getattr(getattr(self, "feed", None), "last_img", None)
        if not live or img is None:
            return None
        H, W = img.shape[:2]
        votes = {}
        for c in live:
            try:
                u, v, kind = float(c[1]), float(c[2]), c[3]
            except (IndexError, TypeError, ValueError):
                continue
            d = dict(left=u, right=W - u, top=v, bottom=H - v)
            side = min(d, key=d.get)
            b, e = votes.get(side, (0, 0))
            votes[side] = (b + (kind == "board"), e + (kind == "edge"))
        boards = [sd for sd, (b, e) in votes.items() if b >= Runner.BOARD_WORDS_MIN and b > e]
        if len(boards) != 2:
            return None
        return dict(x=boards[0], y=boards[1], from_contacts=True)

    def explore_board_sides(self, xs_words, ys_words):
        """(xs, ys) from explore's own map (st setup explore_board_words: the picture edges it found the two boards
        along), matched to this calibration's picture words for each side; None if explore did not find them or they
        do not match one side each."""
        ex = (self.st.get("setup") or {}).get("explore_board_words") or Runner.board_words_from_contacts(self)
        if not ex:
            return None
        blocked = {ex.get("x"), ex.get("y")}
        xc = [s_ for s_ in (0, 1) if xs_words[s_] in blocked]
        yc = [s_ for s_ in (0, 1) if ys_words[s_] in blocked]
        if len(xc) != 1 or len(yc) != 1 or len(blocked) != 2:
            return None
        return xc[0], yc[0]

    def find_boards(self, img):
        """Which two edges are blocked (asked, in picture words; or --board-sides), then their floor lines
        measured in the picture."""
        b0 = PL.Board()
        mid_x, mid_y = (b0.x0 + b0.x1) / 2, (b0.y0 + b0.y1) / 2
        xs_words = {side: CAM.picture_side(self.cam, mid_x, b0.y0 if side == 0 else b0.y1) for side in (0, 1)}
        ys_words = {side: CAM.picture_side(self.cam, b0.x0 if side == 0 else b0.x1, mid_y) for side in (0, 1)}
        auto = Runner.explore_board_sides(self, xs_words, ys_words)
        stored = tuple(int(v) for v in self.a.board_sides.split(",")) if self.a.board_sides else None
        if stored is not None and (auto is None or auto == stored):
            xs, ys = stored
        elif stored is None and auto is not None:
            # coordinator, 2026-10-26 10:45: explore found the blocked boards itself: no questions
            xs, ys = auto
            self.say("Blocked boards from explore's own map (not asked): the x-board along the %s, the y-board along "
                     "the %s of the picture." % (xs_words[xs], ys_words[ys]))
            self.st["setup"]["board_sides_from_explore"] = dict(xs=xs, ys=ys)
        else:
            edges_pic = os.path.join(self.dir, "check_board_edges.png")
            CAM.draw_check(img, self.cam, edges_pic, b0)
            if auto is not None:
                # explore's map and the stored answer disagree: Nick decides, with the picture
                self.say("!!! Explore's own map puts the blocked boards along the %s and the %s of the picture; the "
                         "stored answer (--board-sides %s) says the %s and the %s. Which is right?" % (
                             xs_words[auto[0]], ys_words[auto[1]], self.a.board_sides, xs_words[stored[0]],
                             ys_words[stored[1]]))
                self.st["setup"]["board_sides_disagree"] = dict(explore=list(auto), stored=list(stored))
            self.say("Look at the camera picture (also saved as %s)." % edges_pic)

            # optional (Nick, 2026-10-22 evening: the robots map the board): Enter takes the usual layout (boards
            # at x = max and y = max); the mapping run then says if a board was felt where an open edge was taken
            def pick(question, words, usual=1):
                while True:
                    ans = self.ask("%s Type %s or %s, or just press Enter for %s: " % (
                        question, words[0], words[1], words[usual]), words[usual], image=edges_pic).lower()
                    if ans == "":
                        return usual
                    for side, w in words.items():
                        if ans == w:
                            return side
                    self.say("  Please type %s or %s." % (words[0], words[1]))
            xs = pick("One blocked board runs across the picture along its %s or %s edge (the x-board). Which?"
                      % (xs_words[0], xs_words[1]), xs_words)
            ys = pick("The other blocked board runs along the %s or %s edge (the y-board). Which?"
                      % (ys_words[0], ys_words[1]), ys_words)
            self.st["setup"]["board_sides_answered"] = True
        bd = PL.Board(xboard_side=xs, yboard_side=ys)
        self.say("Blocked boards: the x-board along the %s of the picture, the y-board along the %s."
                 % (xs_words[xs], ys_words[ys]))
        found = {}
        for which in ("x", "y"):
            m = CAM.measure_board_line(img, self.cam, bd, which)
            if m is None:
                edge = (bd.y0 if xs == 0 else bd.y1) if which == "x" else (bd.x0 if ys == 0 else bd.x1)
                self.say("  Could not see the %s-board's line clearly in the picture: using the board's edge (%.1f cm) "
                         "for now; the find check measures it again from where the robots touch it." % (which, edge))
                m = dict(value=edge, spread=None, n=0, values=[], from_edge=True)
            else:
                self.say("  %s-board's floor line at %s = %.2f cm (%d of 17 places, they agree within %.2f cm)"
                         % (which, "y" if which == "x" else "x", m["value"], m["n"], m["spread"]))
                if m["spread"] > 1.0:
                    self.say("  WARNING: the %s-board's line is uneven (%.1f cm): is it straight and flat against the "
                             "floor?" % (which, m["spread"]))
            found[which] = m
        bd.xboard_cm, bd.yboard_cm = found["x"]["value"], found["y"]["value"]
        self.st["board"] = bd.to_dict()
        self.st["setup"]["board_lines_image"] = found
        CAM.draw_check(img, self.cam, os.path.join(self.dir, "check_boards.png"), bd)

    def boards_still(self):
        """Re-measure the board lines (block start): they must not have moved."""
        if self.feed.last_img is None:
            return
        img = self.feed.last_img
        refs = self.st["setup"].get("board_lines_image") or {}
        for which in ("x", "y"):
            if which in getattr(self.board, "missing", ()) or which not in refs:
                continue                      # no board there (--no-boards, or found gone): nothing to re-measure
            m = CAM.measure_board_line(img, self.cam, self.board, which)
            ref0 = refs[which]
            if ref0.get("from_edge"):
                continue
            ref = ref0["value"]
            if m is None:
                self.say("  (could not re-measure the %s-board this time: a robot or a shadow in the way?)" % which)
                continue
            moved = m["value"] - ref
            self.st.setdefault("board_checks", []).append([round(time.time(), 1), which, m["value"], round(moved, 2)])
            if abs(moved) > 1.0:
                self.say("WARNING: the %s-board looks %.1f cm away from where it was at the start. Were the boards "
                         "bumped? (the runs carry on; this is in the data)" % (which, moved))
                self.mark("e02 board %s moved %.1f cm" % (which, moved))

    def robots_ready(self):
        names = self.st["robots"]
        missing = self.wait_robots(names)
        if missing:
            raise SystemExit("Not heard from: %s. Switch them on (same WiFi as this laptop), then start again."
                             % ", ".join(missing))
        bats = self.batteries(names)
        self.start_battery_watch(names)
        if getattr(self.a, "sleep_idle", 0):
            # robots switched on but not in this session: to sleep (firmware 2026-10-25a), checking in every N min
            time.sleep(1.0)
            for n in self.link.robots():
                if n not in names and self.listener.firmware.get(n) in BORDER_FW:
                    self.send(n, "sleep %d" % self.a.sleep_idle)
                    self.asleep_names.add(n)        # battery.csv logs it asleep (the battery learner's sleep drain)
                    self.say("  %s is not in this session: sent to sleep (it checks in every %d min)" % (
                        n, self.a.sleep_idle))
        for n in names:
            b = bats.get(n)
            fw = self.listener.firmware.get(n, "?")
            self.say("%s: connected (firmware %s), battery %s" % (n, fw, "%d%%" % b if b is not None else "not reported"))
            if b is not None and b < BATTERY_WARN:
                self.say("  WARNING: %s's battery is low for a 2-hour session: charge it first if you can." % n)
            if b is not None and b < BATTERY_STOP:
                self.unavailable.add(n)
                self.say("  %s is too flat to run: its runs are put aside." % n)
        self.st["setup"]["batteries"] = bats
        old = [n for n in names if self.listener.firmware.get(n) not in OK_FW + (None,)]
        if old:
            raise SystemExit("%s run%s firmware %s; this session needs %s (it refuses to drive without the edge guard "
                             "and limits blind backing). Upload firmware/home_lab to each robot (Arduino IDE: open "
                             "home_lab.ino, pick the robot's port, press Upload), then start this again." % (
                                 ", ".join(old), "s" if len(old) == 1 else "", "/".join(sorted(
                                     {self.listener.firmware.get(n) for n in old})), " or ".join(OK_FW)))
        self.safety_settings([n for n in names if n not in self.unavailable])
        self.fw_border([n for n in names if n not in self.unavailable])
        self.say("Compass check (`calibrate`, needed once after switching on; spin and roll off: it only wiggles) ...")
        self.calibrate([n for n in names if n not in self.unavailable])
        # every robot found first by a short turn on its spot (nothing leaves its spot, so nothing is mistaken for
        # a robot), THEN parked, each going round the others (2026-10-22, after the first real session)
        if self.st["setup"].get("explore_pending"):
            ok_names = [n for n in names if n not in self.unavailable]
            self.explore(ok_names)
            if getattr(self.a, "dry_explore_only", False) and getattr(self.a, "dry_run", False):
                self.note_end("explore_only", "explore done (--dry-explore-only)")
                raise SystemExit("Explore done (--dry-explore-only): the session ends here.")
            self.explore_tape_check(ok_names)
        else:
            self.find_all([n for n in names if n not in self.unavailable])
        self.park_all([n for n in names if n not in self.unavailable])
        self.active = None
        CAM.draw_check(self.feed.last_img, self.cam, os.path.join(self.dir, "check_robots.png"), self.board,
                       robots={n: tuple(self.st["positions"][n][:2]) for n in names if n in self.st["positions"]})

    # --- explore: the robots calibrate the camera and map the board (Nick, 2026-10-23) ------------------------
    EXPLORE_DANCE = (("go", 15.0), ("turn", 90.0), ("go", 15.0), ("turn", 90.0), ("go", 15.0), ("turn", 90.0),
                     ("go", 15.0), ("turn", 90.0))           # a small square: its corners and turns
    EXPLORE_RAYS = 8                                          # rays per robot, evenly spread in its own frame
    EXPLORE_RAY_CM = 105.0                                    # a ray's longest drive (firmware cap: 110)

    def explore_stop(self, robot, obs, lid=None, contact=None, turn_cmd=None):
        """One observation at a stop: where the camera sees the robot (pixels) and where its own wheels and gyro say
        it is (its own frame, from its last LEG record)."""
        rec = self.listener.leg_final(robot, "-", lid) if lid is not None else None
        if robot in self.held_out and (rec is None or rec.get("state") in ("refused", "dup")):
            # left out by Nick: the leg was never sent, so this is no stop (2026-10-25 15:10: Right's 6 unsent rays
            # went in as stops and "nothing met")
            return None
        res = getattr(self, "explore_resumed", {}).get((robot, lid))
        if res is not None and turn_cmd is not None:
            turn_cmd = res["val"]              # the rest of a turn a hold cut short: that is what it was told
        p = self.settle(min_s=0.4, timeout=3.0)
        lost = None
        if (rec is None or rec.get("state") == "dup") and lid is not None and \
                (robot, lid) in getattr(self, "explore_told", {}) and EXPLORE_LOST_AS_TOLD:
            # (a `dup` with no done record: the leg arrived before, ran, and its own record was lost)
            # 2026-10-26 explore reliability: the leg's final record never arrived (a robot's line lost on the radio;
            # simulator at 10 % loss: 4-8 a run, a 133 deg turn among them). With numbered commands the leg arrived
            # and ran, so a turn is taken as told, and a drive by the camera's move at this robot's own pixels per cm
            # (a ray stops early). Marked "lost": the camera fit takes its length or turn as unknown.
            kind_, val_ = self.explore_told[(robot, lid)]
            lost = dict(kind=kind_, val=val_)
            if kind_ == "turn":
                rec = dict(x_cm=0.0, y_cm=0.0, th_deg=val_, state="lost")
            else:
                rec = dict(x_cm=val_, y_cm=0.0, th_deg=0.0, state="lost")
            self.st.setdefault("setup", {}).setdefault("explore_lost_records", []).append(
                [round(time.time(), 1), robot, lid, kind_, round(val_, 1)])
            self.say("    %s: leg %s's record never arrived: taken as told (%s %.1f), checked by the camera" % (
                robot, lid, kind_, val_), quiet=True)
        if rec is None or rec.get("state") in ("refused", "dup") or None in (num(rec.get("x_cm")), num(rec.get("th_deg"))):
            # the leg did not run (refused, or no record of it): the robot did not move (never an older leg's pose:
            # 2026-10-23 dry run, a refused half turn read as the ray before it and broke Right's whole path)
            rec = dict(x_cm=0.0, y_cm=0.0, th_deg=0.0)
            turn_cmd = None
        x, y, th = (num(rec.get("x_cm")), num(rec.get("y_cm")), num(rec.get("th_deg")))
        if lid is not None and rec.get("state") not in ("refused", "dup", "lost") and None not in (x, y, th):
            # 2026-10-24 Pi plan, stage 3: a robot whose pose was set by camera fixes earlier (a session before,
            # no switch-off) reports its board pose, not the leg's own: "go 12" read as 112 cm, it backed 112 cm
            # and went over the open edge. The leg's own move is always the done pose less the started pose.
            st0 = self.listener.leg_start(robot, "-", lid)
            if st0 is None and rec.get("state") == "done":
                # its 'started' record was lost (WiFi): the final pose alone could be a board pose (stage 3), so the
                # leg counts as what it was told to do (2026-10-24 review)
                v_ = num(rec.get("value")) or 0.0
                x, y, th = (v_, 0.0, 0.0) if rec.get("kind") == "go" else (0.0, 0.0, v_)
                self.say("    %s: leg %s's start record is missing: taken as told (%s %.1f)" % (
                    robot, lid, rec.get("kind"), v_), quiet=True)
            elif st0 is None and EXPLORE_LOST_AS_TOLD and (robot, lid) in getattr(self, "explore_told", {}):
                # 2026-10-26 explore reliability (after-batch run r07): a ray stopped at an edge ("failed") whose start
                # record was lost went in as no move at all (72 cm driven), and the camera's "jump" was then accepted.
                # An exploring robot has no pose, so its final record is the leg's own move: used when it fits what the
                # leg was told, else the told value; either way marked lost (the camera fit takes it as unknown)
                kind_, val_ = self.explore_told[(robot, lid)]
                lost = dict(kind=kind_, val=val_, src="final")
                if kind_ == "go" and abs(x) <= abs(val_) * 1.05 + 1.0 and x * val_ >= 0 and abs(th) < 20.0:
                    pass
                elif kind_ == "turn" and abs(x) < 2.0 and abs(th) <= abs(val_) * 1.2 + 2.0:
                    pass
                else:
                    lost["src"] = "told"
                    x, y, th = (val_, 0.0, 0.0) if kind_ == "go" else (0.0, 0.0, val_)
                self.st.setdefault("setup", {}).setdefault("explore_lost_records", []).append(
                    [round(time.time(), 1), robot, lid, kind_, round(val_, 1), lost["src"]])
            elif st0 is None:
                x = y = th = 0.0               # a failed leg with no start record: how far it got is not known
            else:
                x, y, th = leg_delta(st0, rec)
        if p is None or None in (x, y, th):
            return None
        u, v = self.cam.project((p[0], p[1], EX.ROBOT_TOP_CM))
        # an idle leg's pose is from the leg's own start (x forward, th turned): the leg's distance and turn
        d = x if abs(y) < abs(x) + 1e-9 else math.copysign(math.hypot(x, y), x)
        skip = False
        if obs[robot]:
            bad = self.explore_implausible(obs[robot][-1], d, u, v)
            if bad:
                # the camera's spot does not fit what the wheels did (2026-10-23 dry run: the tracker stayed on
                # Left's old spot for 19 stops while it drove 30-90 cm rays): find it again by a turn on its spot
                # and back (net heading unchanged), and use that. 2026-10-26 (coordinator, 10:02): first by its
                # lights on its own (auto_find, inside hold_for_nick); Nick only if that fails
                self.say("    %s: %s: finding it again" % (robot, bad), quiet=True)
                t_prev = obs[robot][-1].get("t", 0.0)
                spell = any(h[1] == robot and h[0] >= t_prev - 1.0 for h in (self.st.get("holds") or [])) or \
                    any(a_[1] == robot and a_[0] >= t_prev - 1.0 for a_ in (self.st.get("auto_finds") or []))
                self.hold_for_nick(robot, "lost", bad)
                af = self.recently_auto_found(robot)
                if af is not None and math.hypot(af[1][0] - p[0], af[1][1] - p[1]) <= self.AUTO_FIND_SAME_CM \
                        and "jumped" in bad:
                    # its lights confirm the camera's NEW spot after a turn on its spot (which cannot move it 17 cm):
                    # the jump was the camera correcting itself onto the robot, usually after a lost spell (10:02: "it only turned and the camera's
                    # spot jumped 40 px", right after the washed-out Left was found). This stop is right; the one
                    # before, taken during the spell, is not used for the fit
                    obs[robot][-1]["skip"] = True
                    self.say("    %s: its lights confirm the new spot: the stop before (taken while the camera was off "
                             "it) is not used" % robot)
                    self.st.setdefault("explore_jumps_accepted", []).append(
                        [round(time.time(), 1), robot, len(obs[robot]), bad, [round(p[0], 1), round(p[1], 1)], spell])
                    bad = ""
                q = (af[1] if af is not None else (self.identify(robot) if robot not in self.held_out else None)) \
                    if bad else None
                if q is not None:
                    self.tracker.follow(robot, q[0], q[1])
                    self.history.clear()
                    self.tracker.hold(HOLD_CM)
                    p2 = self.settle(min_s=0.3) or q
                    u, v = self.cam.project((p2[0], p2[1], EX.ROBOT_TOP_CM))
                    skip = bool(self.explore_implausible(obs[robot][-1], d, u, v))
                    if skip and abs(d) >= 8.0 and math.hypot(u - obs[robot][-1]["u"], v - obs[robot][-1]["v"]) < 6.0:
                        # found where it was: the tracker was right and the wheels turned without moving it (a
                        # wheel over an edge, or slipping): the leg moved it nowhere
                        self.say("    %s: its wheels turned %.0f cm but it did not move (slip, or a wheel off the "
                                 "board): that leg counts as no move" % (robot, abs(d)))
                        self.st.setdefault("explore_slips", []).append([round(time.time(), 1), robot, round(d, 1)])
                        x = y = d = 0.0
                        skip = False
                elif bad:
                    skip = True
                if bad:
                    self.st.setdefault("explore_refinds", []).append(
                        [round(time.time(), 1), robot, len(obs[robot]), bad, q is not None, skip])
        if turn_cmd is not None:
            th = turn_cmd + ST.wrap180(th - turn_cmd)     # a half turn reads as +-180: keep the side it was told
        if not obs[robot]:
            d, th = 0.0, 0.0                              # the first stop: the frame's origin
        o = dict(t=round(time.time(), 2), x=x, y=y, th=th, d=d, dth=th, u=float(u), v=float(v), lid=lid)
        # its outline's long side in pixels (the robots' own size: a scale cross-check for explore)
        near = [c for c in list(self.tracker.last_cands or []) if c.get("shape") and c.get("rw")
                and math.hypot(c["u"] - u, c["v"] - v) < 12.0]
        if near and not skip:
            c = min(near, key=lambda c: math.hypot(c["u"] - u, c["v"] - v))
            o["size_px"] = round(max(c["rw"], c["rh"]), 2)
        if skip:
            o["skip"] = True                              # its leg still counts for the path; its pixels do not
        if contact:
            o["contact"] = contact
        if res is not None:
            o["d_cut"] = round(res["d_before"], 2)        # how far the same leg went before a hold cut it (fix 3)
        if lost is not None:
            o["lost"] = lost
            ppc = self.explore_px_per_cm(robot, obs)
            if lost["kind"] == "go" and lost.get("src") != "final" and ppc and obs[robot] and \
                    obs[robot][-1].get("u") is not None:
                d_cam = math.hypot(o["u"] - obs[robot][-1]["u"], o["v"] - obs[robot][-1]["v"]) / ppc
                o["d"] = o["x"] = round(math.copysign(min(d_cam, abs(lost["val"]) * 1.05), lost["val"]), 2)
        obs[robot].append(o)
        live = self.st["setup"].setdefault("explore_live", dict(stops=[], contacts=[]))
        live["stops"].append([robot, round(float(u), 1), round(float(v), 1), bool(skip)])
        if contact:
            live["contacts"].append([robot, round(float(u), 1), round(float(v), 1), contact])
        return o

    EXPLORE_MIN_PX_PER_CM = 0.4   # chosen: a camera 6x higher than the provisional 260 cm would still show more
    EXPLORE_STILL_JUMP_PX = 25.0  # chosen: a robot that only turned on its spot moves its top centre a few pixels

    def explore_implausible(self, prev, d, u, v):
        """Why the camera's spot cannot be this robot after a leg of d cm (or '' if it can): it drove but the spot
        did not move (the tracker stayed behind), or it only turned and the spot jumped (the tracker went to
        something else)."""
        moved = math.hypot(u - prev["u"], v - prev["v"])
        if abs(d) >= 8.0 and moved < self.EXPLORE_MIN_PX_PER_CM * abs(d):
            return "it drove %.0f cm and the camera's spot moved %.0f px" % (abs(d), moved)
        if abs(d) < 2.0 and moved > self.EXPLORE_STILL_JUMP_PX:
            return "it only turned and the camera's spot jumped %.0f px" % moved
        return ""

    EXPLORE_RESUME_MAX = 3          # chosen: a leg cut by holds is finished at most this often (no DOE: a loop guard)
    EXPLORE_RESUME_MIN_CM = 1.0     # chosen: less left than this of a drive (or EXPLORE_RESUME_MIN_DEG of a turn) is
    EXPLORE_RESUME_MIN_DEG = 2.0    # not sent (no DOE: below what the robot's own legs resolve; 2026-10-25 15:10 the
                                    # cut turns were 2 deg, 13 deg and 103 deg short)

    def explore_leg(self, robot, kind, val, timeout=None):
        """One idle leg with no camera fix (nothing about the floor is known yet: the robot's own frame must stay as
        it is). Returns (state, info, lid). A leg that a hold stopped ("stop command" with a hold during it) is
        finished after Nick's "carry on" (2026-10-25 15:10: a lost hold stopped Left's 175 deg turn at 72 deg and
        nothing finished it; its return then drove 56 cm the wrong way, at the right edge). The part before the cut
        is an explore stop of its own (obs: self.explore_obs_live); the rest goes as a new leg. Never for a robot
        Nick left out, and never on with a drive towards an edge the camera border held it at."""
        t_from = time.time()
        state, info, lid = self._explore_leg_once(robot, kind, val, timeout)
        d_before = 0.0
        for _ in range(self.EXPLORE_RESUME_MAX):
            if not (state == "failed" and "stop command" in (info or "").lower()):
                break
            while self.hold.is_set() and not self.stop_requested:
                time.sleep(0.2)                                   # Nick's answer first
            cut = [h for h in (self.st.get("holds") or []) if h[0] >= t_from - 0.2] + \
                [[g[0], g[1], "speed"] for g in (self.st.get("speed_guard_stops") or [])
                 if g[0] >= t_from - 0.2 and g[1] == robot]
            if not cut or robot in self.held_out or self.stop_requested:
                break
            if kind == "go" and any(h[1] == robot and h[2] == "border" for h in cut):
                break
            fin = self.listener.leg_final(robot, "-", lid)
            st0 = self.listener.leg_start(robot, "-", lid)
            if fin is None or st0 is None:
                break                                             # how far it got is not known: nothing to finish
            dx, dy, dth = leg_delta(st0, fin)
            if dx is None:
                break
            done = dx if kind == "go" else dth
            rest = val - done
            if abs(rest) < (self.EXPLORE_RESUME_MIN_CM if kind == "go" else self.EXPLORE_RESUME_MIN_DEG) or \
                    rest * val <= 0:
                break
            obs = getattr(self, "explore_obs_live", None)
            if obs is not None and robot in obs:
                self.explore_stop(robot, obs, lid)                # the cut point: a stop of its own
            d_before += dx if kind == "go" else 0.0
            self.say("    %s: a hold cut its %s short (%.1f of %.1f %s): finishing it, %.1f more" % (
                robot, "turn" if kind == "turn" else "drive", done, val, "deg" if kind == "turn" else "cm", rest))
            self.st.setdefault("setup", {}).setdefault("explore_resumed", []).append(
                [round(time.time(), 1), robot, kind, round(val, 1), round(done, 1), round(rest, 1)])
            t_from = time.time()
            to = None if timeout is None else max(10.0, timeout * abs(rest) / max(1e-6, abs(val)))
            state, info, lid = self._explore_leg_once(robot, kind, rest, to)
            if not hasattr(self, "explore_resumed"):
                self.explore_resumed = {}
            self.explore_resumed[(robot, lid)] = dict(val=rest, d_before=d_before)
            val = rest
        return state, info, lid

    def dry_scene_leg(self, robot, kind, val, ctx="explore"):
        """--dry-scene: start this leg's injected event, if any (a camera gap, or a hold with its answer). ctx "leg":
        a leg outside explore (drive_to, parking, finding): kinds "dturn" (a turn of 90 deg or more) and "dgo" (a
        forward drive), so the 15:45 crossings can be re-created after setup."""
        sc = getattr(self, "scene", None)
        if not sc:
            return
        cnt = self.__dict__.setdefault("scene_count", {})
        if ctx == "leg":
            kinds = (["dturn"] if kind == "turn" and abs(val) >= 90.0 else []) + (["dgo"] if kind == "go" and val > 0
                                                                                 else [])
        else:
            kinds = [kind] + (["ray"] if kind == "go" and abs(val - self.EXPLORE_RAY_CM) < 0.01 else []) + \
                (["bigturn"] if kind == "turn" and abs(val) >= 150.0 else [])
        if not kinds:
            return
        for k in kinds:
            cnt[(robot, k)] = cnt.get((robot, k), 0) + 1
        for ev in sc:
            if ev["done"] or ev["robot"] != robot or ev["kind"] not in kinds or cnt[(robot, ev["kind"])] != ev["n"]:
                continue
            ev["done"] = True
            self.st.setdefault("dry_scene", []).append([round(time.time(), 1), robot, kind, round(val, 1), ev["what"]])
            if ev["what"].startswith("gap"):
                secs = float(ev["what"][3:])
                self.say("  (dry run scene: the camera loses %s for %.1f s, %.1f s into its %s %.0f)" % (
                    robot, secs, ev["at"], kind, val), quiet=True)
                self.scene_blind = (time.time() + ev["at"], time.time() + ev["at"] + secs)
            else:
                def fire(ev=ev):
                    # WHAT "noneborder": a border hold nobody answers (the rescue's test: a lost hold is found again
                    # by its lights first, 2026-10-26, so it never reaches the deadline)
                    border = ev["what"] == "noneborder"
                    self.dry_next_answer = "none" if border else ev["what"]
                    self.say("  (dry run scene: a hold during %s's %s %.0f, answer %s)" % (robot, kind, val, ev["what"]))
                    self.hold_for_nick(robot, "border" if border else "lost",
                                       "dry run scene: a %s hold injected" % ("border" if border else "lost"))
                threading.Timer(ev["at"], fire).start()

    def scene_filter(self, s):
        """The camera samples during a --dry-scene gap: none."""
        b = getattr(self, "scene_blind", None)
        if b is not None and b[0] <= time.time() <= b[1]:
            return None
        return s

    EXPLORE_EDGE_TRIES = 2          # chosen: at most this many straight moves away before a turn is given up (no DOE: a
                                    # loop guard; each is up to EXPLORE_EDGE_BACK_CM)

    def camera_edge_room(self, robot):
        """Signed distance (cm) from the robot's centre to the nearest open edge's line (- past it) by the camera now:
        the session's board, or the explore guard's (border_frame). None if the camera does not follow it."""
        if self.tracker is None or self.tracker.active != robot or self.history is None:
            return None
        smp = self.history.since(time.time() - 1.0)
        if len(smp) < 3:
            return None
        x, y = sorted(q[1] for q in smp[-3:])[1], sorted(q[2] for q in smp[-3:])[1]
        f = self.border_frame(x, y)
        if f is None:
            return None
        bd, gx, gy = f
        return min(ST.inward_cm(bd, ax, v, gx if ax == "x" else gy) for ax, v in bd.open_edges())

    def at_edge_why(self, robot):
        """2026-10-26 runner fixes, item 3: words if a turn on the spot is not safe here, else ''. Its own edge guard
        stopped it and it has not backed EXPLORE_EDGE_BACK_CM out since (no turn or forward drive ran), or the camera
        puts its centre within EDGE_TURN_CM of an open edge's line, or past it."""
        t_e = (getattr(self, "edge_stop_t", None) or {}).get(robot)
        backed = (getattr(self, "backed_since_edge", None) or {}).get(robot, 0.0)
        if t_e is not None and backed < EXPLORE_EDGE_BACK_CM - 0.5:
            return "its own edge guard stopped it and it has backed only %.1f cm since" % backed
        room = self.camera_edge_room(robot)
        lim = EXPLORE_TURN_ROOM_CM if getattr(self, "exploring", False) else EDGE_TURN_CM
        if room is not None and room < lim:
            return "the camera puts its centre %s" % ("%.0f cm past an open edge's line" % -room if room < 0 else
                                                      "%.0f cm from an open edge" % room)
        return ""

    def edge_clear_before_turn(self, robot, explore=True):
        """Item 3 (Left's 21:42-21:45 corner: a 90 deg turn on the spot with its nose over the edge swung its body over
        the corner): before a turn on the spot, a robot at or past an open edge moves straight out first. After its
        own edge stop: straight back. Otherwise the way its last straight leg did NOT bring it nearer the edge (the
        camera's room before and after that leg); unknown: no move. True when it is clear (or was), False if not."""
        for k in range(Runner.EXPLORE_EDGE_TRIES + 1):
            why = self.at_edge_why(robot)
            if not why:
                return True
            if k == Runner.EXPLORE_EDGE_TRIES:
                break
            t_e = (getattr(self, "edge_stop_t", None) or {}).get(robot)
            backed = (getattr(self, "backed_since_edge", None) or {}).get(robot, 0.0)
            room = self.camera_edge_room(robot)
            if t_e is not None and backed < EXPLORE_EDGE_BACK_CM - 0.5:
                d = -(EXPLORE_EDGE_BACK_CM - backed)
            else:
                lg = (getattr(self, "last_go", None) or {}).get(robot)
                if not lg or room is None:
                    break
                v_, r0, r1 = lg
                if r0 is not None and r1 is not None and abs(v_) >= 2.0 and abs(r1 - r0) < 0.3 * abs(v_):
                    break                                # it drives along the edge: straight moves do not get it clear
                sign = math.copysign(1.0, v_)
                if r0 is not None and r1 is not None and r1 < r0:
                    sign = -sign                         # that leg brought it nearer: the other way
                lim = EXPLORE_TURN_ROOM_CM if getattr(self, "exploring", False) else EDGE_TURN_CM
                d = sign * min(EXPLORE_EDGE_BACK_CM, max(2.0, lim + EDGE_AWAY_MARGIN_CM - room))
            self.say("    %s: no turn on its spot here (%s): straight %s %.1f cm first" % (
                robot, why, "back" if d < 0 else "forward", abs(d)))
            self.st.setdefault("edge_clear_moves", []).append([round(time.time(), 1), robot, why, round(d, 1)])
            if explore:
                state, info, lid = self._explore_leg_once(robot, "go", d)
                obs = getattr(self, "explore_obs_live", None)
                if obs is not None and robot in obs:
                    self.explore_stop(robot, obs, lid)
            else:
                if robot not in self.moves and not self.begin_moves(robot, "away from the edge"):
                    break
                state, info = self.send_leg(robot, "go %.2f" % d, abs(d) / DRIVE_CMS_GUESS * 2.5 + 6.0)
            if state not in ("done", "dup"):
                break
        self.say("!!! %s: still at the edge (%s): no turn on its spot." % (robot, self.at_edge_why(robot) or "?"))
        self.st.setdefault("edge_turns_refused", []).append([round(time.time(), 1), robot, self.at_edge_why(robot)])
        return False

    def _explore_leg_once(self, robot, kind, val, timeout=None):
        if kind == "turn" and abs(val) >= 1.0 and not self.edge_clear_before_turn(robot):
            # item 3: never a turn on the spot at or past an edge; the turn is not sent (the ray is not driven)
            return "refused", "at an open edge: no turn on its spot", self.last_lid.get(robot)
        room0 = self.camera_edge_room(robot) if kind == "go" else None
        try:
            out = self._explore_leg_inner(robot, kind, val, timeout)
            # what it was told, by leg id: a leg whose final record never arrives is taken from this (explore_stop)
            self.__dict__.setdefault("explore_told", {})[(robot, out[2])] = (kind, float(val))
            return out
        finally:
            if kind == "go":
                self.__dict__.setdefault("last_go", {})[robot] = (val, room0, self.camera_edge_room(robot))

    def _explore_leg_inner(self, robot, kind, val, timeout=None):
        if getattr(self, "scene", None):
            self.dry_scene_leg(robot, kind, val)
        self.moves[robot] = dict(run="-", why="explore")
        est = abs(val) / DRIVE_CMS_GUESS + 1.0 if kind == "go" else abs(val) / TURN_DPS_GUESS + 1.0
        # a forward crawl looks for the edge on purpose (the firmware's edge guard stops it): the virtual border
        # lets it; backwards, or at speed, never
        seek = kind == "go" and val > 0 and (self.speed_now.get(robot) or 99.0) <= EXPLORE_CRAWL_CMS
        if seek:
            self.seek.add(robot)
            self.fw_seek(robot, True)
        try:
            state, info = self.send_leg(robot, "%s %.2f" % (kind, val), timeout or (2.5 * est + 6.0))
        finally:
            if seek:
                self.fw_seek(robot, False)
                # the border's 0.6 s window still holds the crawl towards the edge for a moment after it stops
                # (dry run 2026-10-24: the back-off away from the edge held the session): exempt a moment longer
                self.seek.discard(robot)
                self.seek_end[robot] = time.time() + 0.3     # + the picture's age
        return state, info, self.last_lid.get(robot)

    def explore(self, names):
        """Explore (hybrid): each robot dances a small square near where it stands (turns and straight drives the
        camera can check: gyro scale, wheel balance, a first camera fit), then drives rays out until its front stop
        or edge guard stops it, and back. Every stop pairs pixels with the robot's own pose; e02_explore fits the
        camera, each robot's wheel and gyro scale and where its own frame sits, and the board's sides from the
        rays' ends. One robot moves at a time (the tracker follows one robot); the others stand still."""
        st = self.st
        self.phase = "exploring"
        self.order = "explore"
        self.say("=== Explore: the robots work out the camera and the board (about %d minutes) ===" % max(
            2, round(len(names) * 1.7)))
        self.mark("e02 explore start")
        if self.guard is None:
            self.guard = self.make_guard()
        self.fw_border(names)
        self.find_all(names)
        # backing off an edge it has just felt is the one blind move explore needs, and no camera fix can be sent
        # yet (its pose is unknown): the per-fix allowance is raised for explore and put back after (2026-10-23
        # dry run: every 10 cm back-off after the first was refused)
        for n in names:
            self.send(n, "set e02_back_max_cm %d" % EXPLORE_BACK_ALLOW_CM)
        obs = {n: [] for n in names}
        self.explore_obs_live = obs          # explore_leg's cut-short legs record their cut point here (fix 3)
        t0 = time.time()
        # 1. the room check, every robot first (Nick, 2026-10-24, after two robots went over an open edge: one had
        # been put down 10 cm from it): short slow drives four ways; a robot near an edge works its way in
        room = {}
        for n in names:
            q = st["positions"].get(n)
            if not q or n in self.held_out:
                continue
            self.explore_follow(n)
            state, info, lid = self.explore_leg(n, "turn", 2.0)
            self.explore_stop(n, obs, lid, turn_cmd=2.0)
            room[n] = self.explore_room(n, obs)
            self.explore_park(n)
        # 2. the robot with the most room explores first (Nick, 2026-10-24)
        def room_score(n):
            q = st["positions"].get(n) or (0.0, 0.0)
            others = [tuple(p[:2]) for m, p in st["positions"].items() if m != n and p]
            near = min([math.hypot(q[0] - o[0], q[1] - o[1]) for o in others] or [999.0])
            return (sum(1 for f in room.get(n, []) if f), near)
        order = sorted(room, key=room_score, reverse=True)
        self.say("  exploring in this order (most room first): %s" % ", ".join(order))
        for n in order:
            if n in self.held_out:
                # left out by Nick (a hold during another robot's turn, or its own room check): it stays where it is
                self.say("  %s: left out by Nick: no square, no rays" % n)
                continue
            self.explore_follow(n)
            self.say("  %s: a small square (camera and gyro check) ..." % n)
            self.explore_speed(n, EXPLORE_CRAWL_CMS)
            for kind, val in self.EXPLORE_DANCE:
                state, info, lid = self.explore_leg(n, kind, val)
                if state not in ("done", "dup", "failed") or n in self.held_out:
                    break
                self.explore_stop(n, obs, lid, turn_cmd=val if kind == "turn" else None)
            self.say("  %s: %d rays out until it feels a board or an edge, and back ..." % (n, self.EXPLORE_RAYS))
            self.explore_rays(n, obs, 0.5)
            self.explore_home(n, obs)
            self.moves.pop(n, None)
            if n not in self.held_out:            # (a left-out robot was placed on the map by leave_out)
                p = self.settle(min_s=0.3)
                if p:
                    st["positions"][n] = [p[0], p[1], None]
                    self.tracker.park(n, p[0], p[1])
            st.save()
        st["setup"]["explore_obs"] = obs
        st.save()
        self.say("  fitting the camera and the board from %d stops ..." % sum(len(v) for v in obs.values()))
        try:
            return self.explore_finish(names, obs, t0, last_try=False)
        except ExploreFailed as e:
            return self.explore_fallback(names, str(e))
        except ExploreShort as e:
            # a side no ray met: more rays, between the first ones (Nick puts the robots down anywhere: three near
            # one side leave the far side unmet). 2026-10-26 explore reliability: up to EXPLORE_GAP_ROUNDS rounds
            # (one before), each given first to the robots whose paths the camera fit used (09:55: the gap rays went
            # to Left and Right, both out of the fit, and their contacts did not count)
            why = str(e)
            for rnd in range(EXPLORE_GAP_ROUNDS):
                used = [n for n in ((getattr(self, "_ex_rec", None) or {}).get("used") or []) if n in names]
                self.say("  %s: rays aimed at the sides not met yet (round %d of %d) ..." % (why, rnd + 1,
                                                                                          EXPLORE_GAP_ROUNDS))
                n_gap = self.explore_gap_rays(names, obs, prefer=used)
                # no gap ray could be driven (every sector's ray would pass a robot, or the only robots left are left
                # out): more rays from every robot still in, between its first ones (2026-10-25 15:10: two gap rays
                # given to the left-out Right counted as driven, and this was skipped)
                for n in ((used or names) if not n_gap else []):
                    q = st["positions"].get(n)
                    if not q or n in self.held_out:
                        continue
                    self.tracker.follow(n, q[0], q[1])
                    self.history.clear()
                    self.tracker.hold(HOLD_CM)
                    self.explore_rays(n, obs, 0.0 if rnd == 0 else 0.25)
                    self.explore_home(n, obs)
                    self.moves.pop(n, None)
                    p = self.settle(min_s=0.3)
                    if p:
                        st["positions"][n] = [p[0], p[1], None]
                        self.tracker.park(n, p[0], p[1])
                st["setup"]["explore_obs"] = obs
                st.save()
                try:
                    return self.explore_finish(names, obs, t0, last_try=rnd == EXPLORE_GAP_ROUNDS - 1)
                except ExploreShort as e2:
                    why = str(e2)
                    continue
                except ExploreFailed as e2:
                    return self.explore_fallback(names, str(e2))
            return self.explore_fallback(names, why)

    def explore_fallback(self, names, why):
        """Explore's results failed their checks: never end the session over it (Nick, 2026-10-23 evening). The
        best calibration on hand is used instead (the studio's, or the corner clicks), confirmed the usual way, and
        it is said plainly, here, on every run's line, in the session summary and in the analysis."""
        st = self.st
        self.mark("e02 explore failed")
        self.say("")
        self.say("!!! EXPLORE FAILED: %s." % why)
        self.say("!!! FALLING BACK to a camera calibration not made by the robots (the studio's, the corner clicks, or "
                 "the board's corners). Every run will say so.")
        st["setup"]["explore_failed"] = dict(why=why, t=round(time.time(), 1))
        if getattr(self.a, "dry_explore_only", False) and getattr(self.a, "dry_run", False):
            self.note_end("explore_only", "explore failed (--dry-explore-only): %s" % why)
            st.save()
            raise SystemExit("Explore failed (--dry-explore-only): no fallback, the session ends here.")
        for n in names:
            self.send(n, "set e02_back_max_cm %d" % BACK_ALLOW_CM)
            self.explore_speed(n, EXPLORE_FAST_CMS)
        self.exploring = False
        old_tr = self.tracker
        self.keep_tracker_stats(old_tr)
        self.tracker = None
        if old_tr is not None:
            old_tr.idle()
        st["calibration"] = None
        st["board"] = None
        img = self.feed.wait_frames(5, timeout=8)[-1]
        self.classic_calibration(img)
        st["setup"]["calibration_source"] = dict(kind="fallback", file=st["calibration"], session=False, why=why,
                                                 words="FALLBACK: %s, not made by the robots (explore failed: %s)"
                                                 % (os.path.basename(st["calibration"] or "?"), why))
        st["setup"]["explore_pending"] = False
        st.save()
        self.find_all(names)
        return None

    def calib_note(self):
        """'' when the camera calibration was made this session by the robots; else words saying which older one is
        used (Nick: "make it explicitly clear to me if old calibration data is used to complete a run")."""
        src = (self.st.get("setup") or {}).get("calibration_source") or {}
        if src.get("session"):
            return ""
        doubt = "; Nick said its outline is off" if (self.st.get("setup") or {}).get("calibration_doubted") else ""
        if src.get("kind") == "fallback":
            return "camera calibration: FALLBACK, %s (explore failed%s)" % (os.path.basename(src.get("file") or "?"),
                                                                           doubt)
        cal = self.st.get("calibration")
        return "camera calibration: %s, not made by the robots this session%s" % (
            os.path.basename(cal) if cal else "none", doubt)

    def explore_speed(self, n, cms):
        """e02_cms for the robot's next straight legs (idle go legs drive at it)."""
        if self.speed_now.get(n) != cms:
            self.send(n, "set e02_cms %g" % cms)
            self.speed_now[n] = cms
            time.sleep(0.05)

    def explore_follow(self, n):
        q = self.st["positions"].get(n)
        self.tracker.follow(n, q[0], q[1])
        self.history.clear()
        self.tracker.hold(HOLD_CM)

    def explore_park(self, n):
        self.moves.pop(n, None)
        p = self.settle(min_s=0.3)
        if p:
            self.st["positions"][n] = [p[0], p[1], None]
            self.tracker.park(n, p[0], p[1])
        self.st.save()

    def explore_room(self, n, obs):
        """The room check: a slow EXPLORE_PROBE_CM drive each of four ways and back, a turn of 90 deg between, a
        last turn back to where it faced. Returns [free?] for the four ways. Then, if an open edge was met, it moves
        in the opposite way (if that was free) by EXPLORE_PROBE_CM, slowly. Every leg is an explore stop (more
        data for the camera fit)."""
        self.explore_speed(n, EXPLORE_CRAWL_CMS)
        free, met = [], []
        for k in range(4):
            if k:
                state, info, lid = self.explore_leg(n, "turn", 90.0)
                self.explore_stop(n, obs, lid, turn_cmd=90.0)
            started_in_band = self.in_border_band(n)
            state, info, lid = self.explore_leg(n, "go", EXPLORE_PROBE_CM)
            fin = self.listener.leg_final(n, "-", lid) or {}
            kind = MAP.stop_kind(fin.get("state"), fin.get("info"))
            o = self.explore_stop(n, obs, lid)
            d = abs(o["d"] + o.get("d_cut", 0.0)) if o else 0.0
            free.append(kind is None)
            met.append(kind)
            d = min(d, EXPLORE_PROBE_CM + 2.0)     # never back further than the probe could have gone (stage 3)
            if d > 0.5 and kind is None and (started_in_band or self.in_border_band(n)):
                # a free probe from (or to) a spot inside the virtual border: stay where it got to rather than back
                # blindly into the border (dry runs 2026-10-24: put down 4 and 12 cm from the edge, each back-off
                # held the session); the next probe starts from here
                self.say("    %s: inside the virtual border: not backing towards the edge" % n, quiet=True)
                d = 0.0
            if d > 0.5:
                state, info, lid = self.explore_leg(n, "go", -min(d, EXPLORE_EDGE_BACK_CM if kind else d))
                self.explore_stop(n, obs, lid)
        state, info, lid = self.explore_leg(n, "turn", 90.0)
        self.explore_stop(n, obs, lid, turn_cmd=90.0)
        edges = [k for k in range(4) if met[k] == "edge"]
        self.st.setdefault("setup", {}).setdefault("explore_room", {})[n] = dict(free=free, met=met)
        if edges:
            away = [(k + 2) % 4 for k in edges if free[(k + 2) % 4]]
            if away:
                k = away[0]
                turn = ST.wrap180(90.0 * k)
                if abs(turn) > 150.0:
                    for _ in range(2):
                        state, info, lid = self.explore_leg(n, "turn", turn / 2.0)
                        self.explore_stop(n, obs, lid, turn_cmd=turn / 2.0)
                elif abs(turn) > 1.0:
                    state, info, lid = self.explore_leg(n, "turn", turn)
                    self.explore_stop(n, obs, lid, turn_cmd=turn)
                state, info, lid = self.explore_leg(n, "go", EXPLORE_PROBE_CM)
                self.explore_stop(n, obs, lid)
                self.say("    %s: an open edge within %.0f cm: moved in %.0f cm first" % (n, EXPLORE_PROBE_CM,
                                                                                     EXPLORE_PROBE_CM))
            else:
                self.say("    %s: an open edge within %.0f cm and no free way in: exploring carefully" % (
                    n, EXPLORE_PROBE_CM))
        self.say("    %s: room check: %s" % (n, ", ".join("free" if f else (m or "?") for f, m in zip(free, met))),
                 quiet=True)
        return free

    def explore_ray_blocked(self, n, obs, own_heading, length_cm=None):
        """Would a ray at own_heading (its own frame) pass within EXPLORE_RAY_CLEAR_CM of another robot? Its own
        frame is laid on the provisional floor by its stops so far (a fit with a turn, a size and maybe a mirror).
        length_cm: how far it goes (default a ray's longest); the home and return legs are checked the same way
        (2026-10-25 15:10: Left's unchecked home leg passed 6 cm from Center and pushed it). Returns the other
        robot's name or ''."""
        stops = [q for q in obs[n] if not q.get("skip")]
        if len(stops) < 5:
            return ""
        own = [(p[0], p[1]) for p, q in zip(EX.rebuild(obs[n], 1.0, 1.0), obs[n]) if not q.get("skip")]
        flo = [self.cam.to_floor(q["u"], q["v"], EX.ROBOT_TOP_CM) for q in stops]
        m = EX.own_to_floor_map(own, flo)
        if m is None:
            return ""
        A, t = m
        here = flo[-1]
        d = A @ np.array([math.cos(math.radians(own_heading)), math.sin(math.radians(own_heading))])
        hd = math.degrees(math.atan2(d[1], d[0]))
        others = [(k, tuple(q[:2])) for k, q in self.st["positions"].items() if k != n and q]
        if length_cm is None:
            length_cm = self.EXPLORE_RAY_CM
        else:
            # its own wheels' cm to floor cm (the provisional camera's scale), and the same clearance at its end
            length_cm = length_cm * float(np.sqrt(abs(np.linalg.det(A)))) + EXPLORE_RAY_CLEAR_CM
        return MAP.ray_blocked(here, hd, others, clear_cm=EXPLORE_RAY_CLEAR_CM, length_cm=length_cm)

    def explore_clear_leg(self, n, obs, own_heading, dist):
        """How far (cm, its own frame) a drive at own_heading can go and stay EXPLORE_RAY_CLEAR_CM clear of the
        other robots: dist when clear, else the longest of EXPLORE_LEG_SHORTEN x dist that is, else 0. Returns
        (cm, who): who is the robot in the way ('' when clear)."""
        who = self.explore_ray_blocked(n, obs, own_heading, length_cm=dist)
        if not who:
            return dist, ""
        for f in EXPLORE_LEG_SHORTEN:
            if not self.explore_ray_blocked(n, obs, own_heading, length_cm=f * dist):
                return f * dist, who
        return 0.0, who

    def explore_home(self, n, obs):
        """Back to where it started exploring, on its own wheels and gyro (Nick, 2026-10-23 evening: the robots
        ended explore with a wheel over an open edge, and every compass check after failed). Its own path from its
        legs (no camera: the camera is not calibrated yet); the edge guard and front stop still on."""
        if len(obs[n]) < 2 or n in self.held_out:
            return
        path = EX.rebuild(obs[n], 1.0, 1.0)
        x, y, th = path[-1]
        # a little towards the middle of what it met (its ray ends, in its own frame), from where it started: away
        # from the edges it started near, but not so far that the robots crowd together (2026-10-23 dry run: back
        # at its start, 9 cm from an open edge, it could not park)
        ends = [(p_[0], p_[1]) for p_, q in zip(path, obs[n]) if q.get("contact")]
        tx = ty = 0.0
        if len(ends) >= 3:
            cx, cy = float(np.mean([e[0] for e in ends])), float(np.mean([e[1] for e in ends]))
            k = min(EXPLORE_HOME_IN_CM, math.hypot(cx, cy) * EXPLORE_HOME_FRAC) / max(1e-6, math.hypot(cx, cy))
            tx, ty = cx * k, cy * k
        # its own frame: the target (tx, ty); it stands at (x, y). The straight line there must stay clear of the
        # parked robots (fix 4): else a target up to EXPLORE_HOME_MOVE_CM from it whose line is clear, else as far
        # as is clear, else it stays where it is
        x0_, y0_ = x, y
        x, y = x - tx, y - ty
        dist = math.hypot(x, y)
        if dist < 3.0:
            return
        own_h = math.degrees(math.atan2(-y, -x))
        go_cm, who = self.explore_clear_leg(n, obs, own_h, dist)
        if who:
            best = None
            for r_ in (EXPLORE_HOME_MOVE_CM / 2.0, EXPLORE_HOME_MOVE_CM):
                for k_ in range(8):
                    a_ = math.radians(45.0 * k_)
                    gx, gy = tx + r_ * math.cos(a_), ty + r_ * math.sin(a_)
                    d_ = math.hypot(gx - x0_, gy - y0_)
                    h_ = math.degrees(math.atan2(gy - y0_, gx - x0_))
                    if d_ >= 3.0 and not self.explore_ray_blocked(n, obs, h_, length_cm=d_):
                        if best is None or r_ < best[0]:
                            best = (r_, d_, h_)
                if best is not None:
                    break
            if best is not None:
                self.say("    %s: the way back would pass %s: aimed %.0f cm to one side of its start instead" % (
                    n, who, best[0]), quiet=True)
                dist, own_h, go_cm = best[1], best[2], best[1]
            else:
                self.say("    %s: the way back would pass %s: %s" % (n, who, "only %.0f of %.0f cm driven" % (
                    go_cm, dist) if go_cm >= 3.0 else "it stays where it is"), quiet=True)
            self.st.setdefault("setup", {}).setdefault("explore_legs_checked", []).append(
                [round(time.time(), 1), n, "home", who, round(dist, 1), round(go_cm, 1), best is not None])
            if go_cm < 3.0:
                return
            dist = go_cm
        turn = ST.wrap180(own_h - th)
        if abs(turn) > 150.0:
            state, info, lid = self.explore_leg(n, "turn", turn / 2.0)
            self.explore_stop(n, obs, lid, turn_cmd=turn / 2.0)
            turn /= 2.0
        state, info, lid = self.explore_leg(n, "turn", turn)
        self.explore_stop(n, obs, lid, turn_cmd=turn)
        if n in self.held_out:
            return
        self.explore_speed(n, EXPLORE_CRAWL_CMS)
        state, info, lid = self.explore_leg(n, "go", min(dist, self.EXPLORE_RAY_CM),
                                            timeout=min(dist, self.EXPLORE_RAY_CM) / EXPLORE_CRAWL_CMS * 1.5 + 10.0)
        self.explore_stop(n, obs, lid)
        self.say("    %s: back near where it started exploring, a little towards the middle (%.0f cm by its own wheels)"
                 % (n, dist), quiet=True)

    def explore_creep(self, n, obs, o, fin, k):
        """The front stop ended ray k: creep on to tell an open edge with something beyond it from a blocked side
        (EXPLORE_CREEP_*). o: the ray's stop (its 'board' contact moves to where the creep ends). Returns (kind, cm
        crept): 'edge', 'board' or None (it reached neither: nothing met)."""
        m = re.search(r"front=([0-9.]+)", fin.get("info") or "")
        front0 = float(m.group(1)) if m else FRONT_STOP_CM
        o.pop("contact", None)
        lag = self.camera_lag_s() or 0.0
        cms = max(0.5, min(EXPLORE_CREEP_CMS, EXPLORE_CREEP_UNSEEN_CM / max(lag, 1e-3)))
        self.explore_speed(n, cms)
        self.send(n, "set safe_front_cm %g" % EXPLORE_CREEP_FRONT_CM)
        crept, result, reach = 0.0, None, None
        try:
            while crept < min(EXPLORE_CREEP_MAX_CM, front0 + 2.0) and n not in self.held_out:
                step = min(EXPLORE_CREEP_STEP_CM, EXPLORE_CREEP_MAX_CM - crept)
                state, info, lid = self.explore_leg(n, "go", step, timeout=step / cms * 2.0 + 6.0)
                f2 = self.listener.leg_final(n, "-", lid) or {}
                k2 = MAP.stop_kind(f2.get("state"), f2.get("info"))
                prev = (obs.get(n) or [None])[-1]
                o2 = self.explore_stop(n, obs, lid, contact=k2)
                dd = (o2["d"] + o2.get("d_cut", 0.0)) if o2 is not None else 0.0
                crept += max(0.0, dd)
                stall = Runner.explore_creep_stalled(self, n, obs, prev, o2, dd) if k2 not in ("edge", "board") else None
                if stall is not None:
                    # the wheels turned, the body did not follow: the bumper is against something standing on the
                    # board (a slanted approach touches with the bumper's corner before the front sensor reads
                    # EXPLORE_CREEP_FRONT_CM). A blocked side, met at the bumper; the leg is taken at the camera's
                    # length (explore reliability 2026-10-26: these legs read "crept 11.6 cm and met neither", lost
                    # the contact and put up to 10 cm that never happened into the path)
                    o2["d_own"], o2["d"] = o2["d"], round(math.copysign(stall, o2["d"] or 1.0), 2)
                    o2["contact"] = "board"
                    o2["reach_cm"] = EXPLORE_BUMPER_AHEAD_CM
                    o2["stalled"] = True
                    crept += stall - max(0.0, dd)
                    reach = EXPLORE_BUMPER_AHEAD_CM
                    result = "board"
                    self.st.setdefault("setup", {}).setdefault("explore_creep_stalls", []).append(
                        [round(time.time(), 1), n, k + 1, round(dd, 2), round(stall, 2)])
                    self.say("    %s: creep %d: the wheels went %.1f cm, the camera saw %.1f: against something at "
                             "the bumper" % (n, k + 1, dd, stall), quiet=True)
                    break
                if k2 == "edge":
                    if o2 is not None:
                        o2["cue"] = MAP.edge_cue(f2.get("info"))
                    result = "edge"
                    break
                if k2 == "board":
                    m2 = re.search(r"front=([0-9.]+)", f2.get("info") or "")
                    reach = MAP.TOF_AHEAD_CM + (float(m2.group(1)) if m2 else EXPLORE_CREEP_FRONT_CM)
                    if o2 is not None:
                        o2["reach_cm"] = round(reach, 2)
                    result = "board"
                    break
                if state != "done" or dd < 0.3:
                    break
        finally:
            self.send(n, "set safe_front_cm %g" % FRONT_STOP_CM)
            self.explore_speed(n, EXPLORE_CRAWL_CMS)
        beyond = round(front0 - crept, 1) if result == "edge" else None
        self.st.setdefault("setup", {}).setdefault("explore_creeps", []).append(
            [round(time.time(), 1), n, k + 1, round(front0, 1), round(crept, 1), result, beyond, round(cms, 2)])
        if result == "edge":
            self.say("    ray %d: the front saw something %.0f cm ahead, but the floor ended first (after %.1f cm): an "
                     "OPEN edge; the object stands beyond it, about %.0f cm on (not a board side)" % (
                         k + 1, front0, crept, max(0.0, beyond or 0.0)), quiet=True)
        elif result == "board":
            self.say("    ray %d: crept %.1f cm up to what the front saw, floor all the way: a blocked side" % (
                k + 1, crept), quiet=True)
        else:
            self.say("    ray %d: the front saw something %.0f cm ahead; crept %.1f cm and met neither" % (
                k + 1, front0, crept), quiet=True)
        return result, crept

    def explore_px_per_cm(self, n, obs):
        """The camera's pixels per cm of this robot's own wheel travel, from its straight legs of 8 cm or more so far
        (the median: a local scale that needs no camera fit). None with fewer than 3 such legs."""
        r = []
        st = obs.get(n) or []
        for a_, b_ in zip(st[:-1], st[1:]):
            d_ = abs(b_.get("d") or 0.0)
            if d_ >= 8.0 and abs(b_.get("dth") or 0.0) < 3.0 and not a_.get("skip") and not b_.get("skip") and \
                    not b_.get("stalled"):
                r.append(math.hypot(b_["u"] - a_["u"], b_["v"] - a_["v"]) / d_)
        return float(np.median(r)) if len(r) >= 3 else None

    def explore_creep_stalled(self, n, obs, prev, o2, dd):
        """A creep step whose camera move is under EXPLORE_STALL_SHARE of what its wheels drove: the cm the body
        really moved (by the robot's own pixels per cm), else None."""
        if prev is None or o2 is None or dd < 1.5 or prev.get("u") is None or o2.get("u") is None:
            return None
        ppc = Runner.explore_px_per_cm(self, n, obs)
        if not ppc:
            return None
        moved = math.hypot(o2["u"] - prev["u"], o2["v"] - prev["v"]) / ppc
        if moved < EXPLORE_STALL_SHARE * dd and dd - moved >= 1.0:
            return moved
        return None

    def explore_rays(self, n, obs, offset):
        """EXPLORE_RAYS rays for robot n, at (k + offset) x 360 / EXPLORE_RAYS deg in its own frame, each out until
        its front stop or edge guard stops it (or the ray's length) and back."""
        for k in range(self.EXPLORE_RAYS):
            if n in self.held_out:
                left = self.EXPLORE_RAYS - k
                self.say("    %s: left out by Nick: its last %d ray(s) not driven" % (n, left), quiet=True)
                self.st.setdefault("setup", {}).setdefault("explore_rays_not_driven", []).append([n, k, left])
                return
            # evenly spread rays in its own frame (its heading from its own legs: 2026-10-23 dry run 5, a fixed
            # turn between rays after the half turn back left a 93 deg gap, and no robot met one side)
            self.explore_one_ray(n, obs, (k + offset) * 360.0 / self.EXPLORE_RAYS, k)

    def explore_one_ray(self, n, obs, want, k):
        """One ray of robot n at heading `want` (deg, its own frame): turn, out until a board or an edge stops it
        (or the ray's length), back off an edge, and back the way it came. Returns the contact kind, '' (driven,
        nothing met) or None (not driven: it would pass another robot, or Nick left it out)."""
        if n in self.held_out:
            return None
        own_h = sum(q["dth"] for q in obs[n][1:])
        turn = ST.wrap180(want - own_h)
        if abs(turn) > 150.0:          # two turns: a half turn is ambiguous (+-180 alike)
            state, info, lid = self.explore_leg(n, "turn", turn / 2.0)
            self.explore_stop(n, obs, lid, turn_cmd=turn / 2.0)
            turn = turn / 2.0
        state, info, lid = self.explore_leg(n, "turn", turn)
        self.explore_stop(n, obs, lid, turn_cmd=turn)
        if n in self.held_out:
            return None
        who = self.explore_ray_blocked(n, obs, want)
        if who:
            self.say("    ray %d: skipped (it would pass %s)" % (k + 1, who), quiet=True)
            self.st.setdefault("setup", {}).setdefault("explore_skipped_rays", []).append([n, k, round(want, 1), who])
            return None
        self.explore_speed(n, EXPLORE_CRAWL_CMS)
        state, info, lid = self.explore_leg(n, "go", self.EXPLORE_RAY_CM,
                                            timeout=self.EXPLORE_RAY_CM / EXPLORE_CRAWL_CMS * 1.5 + 10.0)
        fin = self.listener.leg_final(n, "-", lid) or {}
        kind = MAP.stop_kind(fin.get("state"), fin.get("info"))
        o = self.explore_stop(n, obs, lid, contact=kind)
        if o is not None:
            o["ray"] = True                   # a ray's out leg (the fit's leg repair looks at these: e02_explore)
        if o is not None and kind == "edge":
            o["cue"] = MAP.edge_cue(fin.get("info"))
        if n in self.held_out:
            # a hold stopped the ray and Nick left the robot out: it stays where it is (not "nothing met")
            self.say("    ray %d: stopped by a hold; Nick left %s out" % (k + 1, n), quiet=True)
            return None
        creep_cm = 0.0
        if kind == "board" and o is not None:
            kind, creep_cm = self.explore_creep(n, obs, o, fin, k)
            if n in self.held_out:
                return None
        self.say("    ray %d: %s" % (k + 1, {"edge": "an open edge", "board": "a board (or a robot)"}.get(
            kind, "nothing met")), quiet=True)
        d_out = (o["d"] + o.get("d_cut", 0.0) + creep_cm) if o is not None else 0.0
        if o is not None and kind and abs(d_out) < 2.0:
            o.pop("contact", None)            # it was already there (it never left the edge): not a new side point
        if kind == "edge":
            # back off the edge first, blind (2026-10-23 real board: at an open edge the edge guard refused every
            # turn after it, the robot stayed with a wheel over the edge, and the rest of its rays were lost)
            state, info, lid = self.explore_leg(n, "go", -EXPLORE_EDGE_BACK_CM)
            bo = self.explore_stop(n, obs, lid)
            if o is not None and bo is not None:
                o["d_back"] = bo["d"]
        back = abs(d_out) + (o.get("d_back") or 0.0) if o else 0.0
        back = min(back, self.EXPLORE_RAY_CM + EXPLORE_EDGE_BACK_CM)   # never further than the ray could go (stage 3)
        if back > 1.0 and n not in self.held_out:
            # back the way it came: (nearly) a half turn, then forward (175 deg: a 180 is ambiguous, the robot
            # sometimes goes round the other way and reports +-180 alike: 2026-10-23 dry run); (backing up is blind and needs camera fixes,
            # which explore can't give yet; forward has the front stop and the edge guard)
            state, info, lid = self.explore_leg(n, "turn", 175.0)
            self.explore_stop(n, obs, lid, turn_cmd=175.0)
            if n in self.held_out:
                return kind or ""
            # the way back checked like a ray (fix 4): a 175 deg turn and gyro drift do not bring it back on the
            # same line (2026-10-25 15:10: ray 8's return drifted about 11 deg)
            own_h = sum(q["dth"] for q in obs[n][1:])
            go_cm, who = self.explore_clear_leg(n, obs, own_h, back)
            if who:
                self.say("    ray %d: the way back would pass %s: %s" % (k + 1, who, "only %.0f of %.0f cm" % (
                    go_cm, back) if go_cm >= 3.0 else "it stays where it is"), quiet=True)
                self.st.setdefault("setup", {}).setdefault("explore_legs_checked", []).append(
                    [round(time.time(), 1), n, "return", who, round(back, 1), round(go_cm, 1), False])
            if go_cm >= 3.0:
                self.explore_speed(n, EXPLORE_FAST_CMS)          # back over ground it has just driven
                state, info, lid = self.explore_leg(n, "go", go_cm, timeout=40.0)
                self.explore_stop(n, obs, lid)
        return kind or ""

    EXPLORE_SECTORS = 8         # chosen: 45 deg sectors round the middle of all stops seen
    EXPLORE_SECTOR_MIN = 2      # a sector with fewer ray ends than this is a gap (rect_check wants 2 a side)

    def explore_gaps(self, names, obs):
        """Where the rays have not met anything yet: 45 deg sectors round the middle of every stop seen (on the
        provisional floor) with fewer than EXPLORE_SECTOR_MIN ray ends. Returns [(sector heading deg, count)]."""
        flo_all, ends = [], []
        for n in names:
            for q in obs.get(n, []):
                if q.get("skip"):
                    continue
                f = self.cam.to_floor(q["u"], q["v"], EX.ROBOT_TOP_CM)
                if not (np.isfinite(f[0]) and np.isfinite(f[1])):
                    continue
                flo_all.append(f)
                if q.get("contact"):
                    ends.append(f)
        if len(flo_all) < 5:
            return []
        cx, cy = float(np.mean([f[0] for f in flo_all])), float(np.mean([f[1] for f in flo_all]))
        w = 360.0 / self.EXPLORE_SECTORS
        cnt = [0] * self.EXPLORE_SECTORS
        for e in ends:
            a = math.degrees(math.atan2(e[1] - cy, e[0] - cx)) % 360.0
            cnt[int(a // w) % self.EXPLORE_SECTORS] += 1
        return [((k + 0.5) * w, cnt[k]) for k in range(self.EXPLORE_SECTORS) if cnt[k] < self.EXPLORE_SECTOR_MIN], (cx, cy)

    def explore_gap_rays(self, names, obs, prefer=None):
        """2026-10-24 (robots put down scattered: rays skipped because they would pass another robot left a side
        unmet, and explore fell back): rays aimed at the gaps. For each sector the rays have not met, the robot
        whose ray there is clear of the others (on the provisional floor) drives one, aimed out from the middle of
        everything seen. Returns how many gap rays were driven."""
        got = self.explore_gaps(names, obs)
        if not got:
            return 0
        gaps, (cx, cy) = got
        if not gaps:
            return 0
        done = 0
        self.say("  %d sector(s) of the board no ray has met yet: rays aimed at them ..." % len(gaps))
        for hd, c in gaps:
            best = None
            # the robots the camera fit used first; the others only when none of those has a clear ray there
            order = [m for m in names if m in (prefer or [])]
            order = order + [m for m in names if m not in order]
            for n in order:
                if best is not None and prefer and n not in prefer and best[1] in prefer:
                    break
                if n in self.held_out or n in self.unavailable:
                    continue                  # left out by Nick: never a gap ray (2026-10-25 15:10: both top rays)
                stops = [q for q in obs.get(n, []) if not q.get("skip")]
                if len(stops) < 5:
                    continue
                own = [(p[0], p[1]) for p, q in zip(EX.rebuild(obs[n], 1.0, 1.0), obs[n]) if not q.get("skip")]
                flo = [self.cam.to_floor(q["u"], q["v"], EX.ROBOT_TOP_CM) for q in stops]
                m = EX.own_to_floor_map(own, flo)
                if m is None:
                    continue
                A, _ = m
                here = flo[-1]
                # aim from where it stands towards the sector's far side (out from the middle)
                tx, ty = cx + 200.0 * math.cos(math.radians(hd)), cy + 200.0 * math.sin(math.radians(hd))
                fh = math.degrees(math.atan2(ty - here[1], tx - here[0]))
                others = [(k_, tuple(q[:2])) for k_, q in self.st["positions"].items() if k_ != n and q]
                if MAP.ray_blocked(here, fh, others, clear_cm=EXPLORE_RAY_CLEAR_CM, length_cm=self.EXPLORE_RAY_CM):
                    continue
                try:
                    d = np.linalg.solve(A, np.array([math.cos(math.radians(fh)), math.sin(math.radians(fh))]))
                except np.linalg.LinAlgError:
                    continue
                own_h = math.degrees(math.atan2(d[1], d[0]))
                # the robot nearest the gap goes
                dist = math.hypot(tx - here[0], ty - here[1])
                if best is None or dist < best[0]:
                    best = (dist, n, own_h)
            if best is None:
                self.say("    sector at %.0f deg: every robot's ray there would pass another robot" % hd, quiet=True)
                continue
            _, n, own_h = best
            q = self.st["positions"].get(n)
            if q:
                self.tracker.follow(n, q[0], q[1])
                self.history.clear()
                self.tracker.hold(HOLD_CM)
            kind = self.explore_one_ray(n, obs, own_h, 99)
            self.st.setdefault("setup", {}).setdefault("explore_gap_rays", []).append(
                [n, round(hd, 1), round(own_h, 1), kind])
            if kind is not None:
                done += 1                     # only a ray that was driven counts (a skipped or left-out one is not)
            self.moves.pop(n, None)
            if n not in self.held_out:
                p = self.settle(min_s=0.3)
                if p:
                    self.st["positions"][n] = [p[0], p[1], None]
                    self.tracker.park(n, p[0], p[1])
        return done

    def heavy(self, fn, *args, **kw):
        """A long calculation (explore's camera fit: 11 s here, 25-28 s on the Pi) in the helper process
        (e02_offload), so the camera loop keeps its pictures a second (2026-10-25 15:10: 24 -> 3.8 during the
        fits). --fits-in-runner: in the runner, as before (the before/after measurement)."""
        if getattr(getattr(self, "a", None), "fits_in_runner", False):
            return fn(*args, **kw)
        if getattr(self, "offload", None) is None:
            self.offload = OFF.Worker(log=lambda m: self.say(m))
        t0 = time.time()
        self.mark("e02 heavy start %s" % fn.__name__)
        try:
            return self.offload.call(fn, *args, **kw)
        finally:
            self.mark("e02 heavy end %s %.1f s" % (fn.__name__, time.time() - t0))
            self.st.setdefault("heavy_calls", []).append([round(t0, 1), round(time.time() - t0, 2), fn.__name__,
                                                          "runner" if self.offload.failed else "helper"])

    def explore_finish(self, names, obs, t0, last_try=True):
        """_explore_finish, with what each try found kept in the state (setup.explore_tries), passed or not: the
        fitted camera, the robots used and left out, the contacts and why it failed (2026-10-26 explore reliability
        study: failed tries kept nothing, so their camera could not be scored against the truth afterwards)."""
        self._ex_rec = rec = dict(t=round(time.time(), 1), last_try=last_try, minutes=round((time.time() - t0) / 60.0, 1))
        try:
            return self._explore_finish(names, obs, t0, last_try)
        except BaseException as e:
            rec["why"] = "%s: %s" % (type(e).__name__, e)
            raise
        finally:
            self.st.setdefault("setup", {}).setdefault("explore_tries", []).append(rec)
            self.st.save()

    def _explore_finish(self, names, obs, t0, last_try=True):
        st = self.st
        rec = getattr(self, "_ex_rec", None)
        if rec is None:
            rec = {}
        robots = {n: v for n, v in obs.items() if len(v) >= 5}
        if not robots:
            raise SystemExit("Explore: too few stops seen by the camera to work anything out. Start again with "
                             "--classic-setup to use a calibration file.")
        size = (self.feed.last_img.shape[1], self.feed.last_img.shape[0])
        # check before trust: each robot's own path against its own pixels, alone (one robot's bad data, a wheel
        # off the board or the tracker on something else, must not bend everyone's calibration)
        checks = {n: self.heavy(EX.robot_check, v) for n, v in robots.items()}
        st["setup"]["explore_robot_checks"] = checks
        for n, c in checks.items():
            self.say("  %s: %s" % (n, "its camera path fits its own wheels and gyro within %.1f px (gyro scale %.3f): "
                                      "used" % (c["rms_px"], c["g"]) if c["ok"] else
                                   "LEFT OUT of the camera calibration: %s" % c["why"]))
        left_out = [n for n in robots if not checks[n]["ok"]]
        robots = {n: v for n, v in robots.items() if checks[n]["ok"]}
        if not robots:
            raise ExploreFailed("no robot's camera path fitted its own wheels and gyro")
        sol = self.heavy(EX.fit, robots, size)
        # then each robot against the joint fit: one whose stops the shared camera misses by more than
        # FIT_ROBOT_MAX_PX is left out and the rest fitted again (once)
        worse = [n for n, v in sol["per_robot_rms_px"].items() if not (v <= EX.FIT_ROBOT_MAX_PX)]
        if worse and len(worse) < len(robots):
            for n in worse:
                self.say("  %s: LEFT OUT of the camera calibration: the shared camera misses its stops by %.1f px" % (
                    n, sol["per_robot_rms_px"][n]))
                checks[n] = dict(checks[n], ok=False, why="joint fit %.1f px" % sol["per_robot_rms_px"][n])
                left_out.append(n)
            robots = {n: v for n, v in robots.items() if n not in worse}
            sol = self.heavy(EX.fit, robots, size)
        rec.update(sol=dict(f=sol["f"], k1=sol["k1"], rvec=list(sol["rvec"]), tvec=list(sol["tvec"]), size=list(sol["size"]),
                            s=dict(sol["s"]), g=dict(sol["g"]), frames={k_: list(v_) for k_, v_ in sol["frames"].items()},
                            rms_px=sol["rms_px"], per_robot_rms_px=dict(sol["per_robot_rms_px"]), n=sol["n"]),
                   used=list(robots), left_out=list(left_out),
                   checks={k_: dict(ok=v_["ok"], why=v_["why"], rms_px=v_["rms_px"]) for k_, v_ in checks.items()})
        bad = EX.fit_check(sol) or ("forced to fail (--dry-explore-fail)" if getattr(self.a, "dry_explore_fail", False)
                                    and getattr(self.a, "dry_run", False) else "")
        if not bad and len(robots) < EXPLORE_MIN_ROBOTS and len(obs) >= EXPLORE_MIN_ROBOTS:
            # 2026-10-26 09:55 session: one robot's path alone set the camera's scale, with nothing to confirm it
            # (its own wheel scale and the camera's size trade off). With a second robot's path the two must agree
            bad = "only %s's path fits its own wheels and gyro: one robot cannot confirm the camera's scale" % (
                ", ".join(robots))
        if bad:
            self.say("  camera fit: %.1f px rms; wheel scales %s; gyro scales %s" % (
                sol["rms_px"], ", ".join("%s %.3f" % kv for kv in sol["s"].items()),
                ", ".join("%s %.3f" % kv for kv in sol["g"].items())))
            raise ExploreFailed(bad)
        # the legs the fit found the wheels had overcounted (a creep against a board, a hang at an edge) are taken at the
        # camera's length here too: the contacts and the robots' end places are built on these paths
        robots = EX.apply_repairs(robots, sol)
        rec["repaired"] = sum(len(v) for v in (sol.get("repairs") or {}).values())
        pts = EX.contacts(robots, sol, others={n: obs[n] for n in left_out if n in obs})
        if left_out and EXPLORE_SEEN_CONTACTS:
            # the left-out robots' own rays still met sides: placed by the fitted camera (09:55: 8 board contacts by
            # Left and Right's gap rays went uncounted and the sides never closed)
            paths = {n: [(q.get("t"), p_[0], p_[1]) for p_, q in zip(EX.to_world(EX.rebuild(v, sol["s"][n], sol["g"][n]),
                                                                              sol["frames"][n]), v)]
                     for n, v in robots.items()}
            seen = EX.contacts_seen({n: obs[n] for n in left_out if n in obs}, sol, paths)
            rec["seen_contacts"] = len(seen)
            pts = pts + seen
        sd = pts
        rect = EX.rectangle_from_rays(pts)
        edge_ahead = None
        if rect:
            # where the edge guard really sees an edge, from the rays' slants (most stops are the distance sensor's
            # bottom zone, which looks ahead of the line sensors)
            rect0 = rect
            rect, edge_ahead = EX.refine_edges(rect, MAP.BOTTOM_AHEAD_CM)
            if edge_ahead is not None and EX.rect_check(rect) and not EX.rect_check(rect0):
                # 2026-10-26 explore reliability (after-batch run r04): the learned reach moved an open edge 7.6 cm and
                # its ray ends no longer agreed, where the sides as met agreed: the sides as met are kept
                self.say("  (the edge guard's learned reach (%.1f cm) would break the sides: not used)" % edge_ahead,
                         quiet=True)
                rec["refine_dropped"] = round(edge_ahead, 2)
                rect, edge_ahead = rect0, None
        rect_bad = EX.rect_check(rect) if rect else ""
        rec.update(contacts=[[round(c_[0], 1), round(c_[1], 1), c_[2], round(c_[3], 1)] for c_ in pts],
                   rect=None if not rect else {k_: [round(v_[0], 2), v_[1]["kind"], v_[1]["n"]] for k_, v_ in
                                               rect["lines"].items()},
                   rect_angle=None if not rect else round(rect["angle_deg"], 2), rect_bad=rect_bad)
        if rect_bad:
            self.say("  the sides from the rays don't hold together yet: %s" % rect_bad)
            rect = None
        bf = EX.board_frame(rect) if rect else None
        rep = dict(rms_px=round(sol["rms_px"], 2), f_px=round(sol["f"], 1), k1=round(sol["k1"], 4),
                   s={n: round(v, 4) for n, v in sol["s"].items()}, g={n: round(v, 4) for n, v in sol["g"].items()},
                   n=sol["n"], per_robot_rms_px={n: round(v, 2) for n, v in sol["per_robot_rms_px"].items()},
                   contacts=len(pts), sides=len(sd), minutes=round((time.time() - t0) / 60.0, 1),
                   edge_ahead_cm=None if edge_ahead is None else round(edge_ahead, 2),
                   outliers=sol.get("outliers", 0), size_check=EX.size_scale(robots, sol, size), left_out=left_out)
        st["setup"]["explore"] = rep
        self.say("  camera fit: %.1f px rms over %d stops; wheel scales %s; gyro scales %s" % (
            sol["rms_px"], sol["n"], ", ".join("%s %.3f" % kv for kv in sol["s"].items()),
            ", ".join("%s %.3f" % kv for kv in sol["g"].items())))
        sc = rep.get("size_check")
        if sc:
            self.say("  scale check by the robots' own size: their outlines come out %.1f%% %s than %.1f cm (%d stops; "
                     "reported, not applied: the outline includes some shadow)" % (
                         abs(100 * (sc["ratio"] - 1)), "longer" if sc["ratio"] > 1 else "shorter", EX.ROBOT_LENGTH_CM,
                         sc["n"]))
        if edge_ahead is not None:
            self.say("  the edge guard sees an open edge %.1f cm ahead of the axle (learned from the rays' slants)"
                     % edge_ahead)
        if bf is not None and EXPLORE_SIZE_CHECK_PCT:
            # 2026-10-26 explore reliability: the board's known size as a last check (the plywood sheet, 122 x 122 cm,
            # e02_plan.Board). After-batch run r07 "found" a board 15 % short on one side from a camera 8 % small; a
            # wrong board is worse than none (the fallback says plainly what it uses)
            kb = PL.Board()
            kw, kh = kb.x1 - kb.x0, kb.y1 - kb.y0
            off = max(abs(bf["W"] / kw - 1.0), abs(bf["H"] / kh - 1.0)) * 100.0
            rec["size_off_pct"] = round(off, 1)
            if off > EXPLORE_SIZE_CHECK_PCT:
                self.say("  the board from the rays is %.1f x %.1f cm, %.0f %% off the plywood's %.0f x %.0f cm: not trusted"
                         % (bf["W"], bf["H"], off, kw, kh))
                bf = None
        if bf is None and not last_try:
            st.save()
            raise ExploreShort("the rays did not show all four sides as two open edges and two boards")
        if bf is None:
            st.save()
            raise ExploreFailed("the rays did not show all four sides as two open edges and two boards (%d points)"
                                % len(pts))
        rec.update(W=bf["W"], H=bf["H"], bf_R=np.asarray(bf["R"]).tolist(), bf_t=np.asarray(bf["t"]).tolist())
        rep.update(W=round(bf["W"], 2), H=round(bf["H"], 2), kinds=bf["kinds"], spreads={
            k: round(v, 2) for k, v in bf["spreads"].items()}, side_points=bf["n"])
        try:
            # coordinator, 2026-10-26 10:45: explore found the blocked boards itself; kept in picture words at once, so
            # that a fallback afterwards (a later check failing) takes them instead of asking Nick (find_boards)
            cbx = EX.camera_in_board(sol, bf)
            import cameras as CMS_
            cam_ex = CMS_.CameraModel(cbx["K"], cbx["dist"], cbx["rvec"], cbx["tvec"], cbx["image_size"], name="ex")
            st["setup"]["explore_board_words"] = dict(
                x=CAM.picture_side(cam_ex, bf["W"] / 2.0, bf["H"]), y=CAM.picture_side(cam_ex, bf["W"], bf["H"] / 2.0),
                t=round(time.time(), 1))
        except Exception as e:
            self.say("  (explore's board sides in picture words not kept: %s)" % e, quiet=True)
        self.say("  the board: %.1f x %.1f cm (two open edges, two boards); sides fitted within %s cm" % (
            bf["W"], bf["H"], "/".join("%.1f" % v for v in bf["spreads"].values())))
        # the camera, in the board frame; saved as this session's calibration
        cb = EX.camera_in_board(sol, bf)
        import cameras as CMS
        model = CMS.CameraModel(cb["K"], cb["dist"], cb["rvec"], cb["tvec"], cb["image_size"], name="overhead",
                                fit=dict(method="explore (robots)", rms_px=round(sol["rms_px"], 2), n=sol["n"]))
        path = os.path.join(self.dir, "cameras", "overhead.json")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        model.save(path)
        st["calibration"] = path
        bd = PL.Board(rect=(0.0, 0.0, bf["W"], bf["H"]), xboard_side=1, yboard_side=1, xboard_cm=bf["H"],
                      yboard_cm=bf["W"])
        st["board"] = bd.to_dict()
        st["setup"]["board_lines_image"] = {"x": dict(value=bf["H"], from_explore=True),
                                            "y": dict(value=bf["W"], from_explore=True)}
        st["setup"]["map"] = dict(robot="all", from_explore=True, t=round(time.time(), 1),
                                  sides={k: dict(kind=bf["kinds"][k], n=bf["n"][k], spread=bf["spreads"][k])
                                         for k in bf["kinds"]})
        # robots: where they are now, and which way they face, in the board frame
        self.exploring = False
        self.cam = CAM.load_camera(path)
        self.board = bd
        old_tr = self.tracker
        self.keep_tracker_stats(old_tr)
        self.tracker = None
        self.make_tracker()
        self.tracker_area()
        if old_tr is not None:
            old_tr.idle()
        for n, stops in robots.items():
            x, y, th = EX.to_world(EX.rebuild(stops, sol["s"][n], sol["g"][n]), sol["frames"][n])[-1]
            bx, by = EX.to_board(bf, x, y)
            hb = math.degrees(math.atan2(*(bf["R"] @ np.array([math.cos(math.radians(th)),
                                                               math.sin(math.radians(th))]))[::-1]))
            st["positions"][n] = [bx, by, None]
            self.heads[n] = [ST.wrap180(hb), True]
        self.fresh_floor()
        for n in robots:
            q = st["positions"][n]
            self.tracker.follow(n, q[0], q[1])
            self.history.clear()
            self.tracker.hold(HOLD_CM)
            self.fix_now(n, self.heads[n][0])              # the robot's own frame becomes the board's
            self.tracker.park(n, q[0], q[1])
        for n in names:
            self.send(n, "set e02_back_max_cm %d" % BACK_ALLOW_CM)
            self.explore_speed(n, EXPLORE_FAST_CMS)
        st["setup"]["explore_pending"] = False
        st["setup"]["calibration_source"] = dict(kind="explore", file=path, session=True,
                                                 words="made by the robots this session (explore)")
        st.save()
        if left_out:
            self.say("  finding %s with the new calibration (left out of it) ..." % ", ".join(left_out))
            self.find_all(left_out)
        self.mark("e02 explore end")
        self.say("  explore done in %.1f min: the camera is calibrated and the board mapped by the robots." % rep["minutes"])
        self.fw_border(names)                 # its own board now: the robots' own border follows it
        return rep

    def find_all(self, names):
        """Identify every robot (a turn on its spot), then a fresh floor memory with all of them known."""
        self.phase = "finding"
        self.say("Finding the robots: each turns a little on its spot and back (about 6 s each) ...")
        for n in names:
            self.st["positions"].pop(n, None)
        for n in names:
            p = self.identify(n)
            self.moves.pop(n, None)
            if p is None:
                self.say("  The camera did not see %s turn." % n)
                self.help_from_nick(n, "the camera did not see it turn when asked to", park=False)
                continue
            self.st["positions"][n] = [p[0], p[1], None]
            self.say("  %s is at (%.1f, %.1f) cm" % (n, p[0], p[1]))
        self.st.save()
        self.fresh_floor()

    CAMERA_WAIT_S = 300.0     # chosen: long enough to restart the Pi's recorder by hand

    def wait_for_camera(self):
        """No pictures (2026-10-23 evening: the laptop slept, the SSH window closed and with it the Pi's recorder):
        wait for the camera to come back, saying so every 30 s; if it does not, save and stop cleanly with how to
        carry on (never a crash). Returns the new pictures."""
        self.say("  NO PICTURES from the overhead camera. If the PI window closed, start the recorder again there:\n"
                 "    python3 tools/pi/camrec.py --device 0 --names overhead --fps 25 --label E02 --no-lock\n"
                 "  Waiting up to %d minutes ..." % round(self.CAMERA_WAIT_S / 60))
        self.mark("e02 camera lost")
        end = time.time() + self.CAMERA_WAIT_S
        while time.time() < end:
            self.camera_alive()
            frames = self.feed.wait_frames(5, timeout=30)
            if frames:
                self.say("  The camera is back.")
                return frames
            self.say("  ... still no pictures (%d s left)" % max(0, round(end - time.time())))
        self.st.save()
        raise SystemExit("The overhead camera did not come back. Everything so far is saved; start the recorder on "
                         "the Pi, then run the same command again and type y to carry on this session.")

    def fresh_floor(self):
        """A new floor memory from a few still pictures; every robot the runner knows of is marked on it."""
        frames = self.feed.wait_frames(10, timeout=8)
        if not frames:
            frames = self.wait_for_camera()
        known = {n: tuple(q[:2]) for n, q in self.st["positions"].items() if q}
        self.tracker.idle()
        self.tracker.clear_protect()
        self.tracker.set_background(frames, robots=list(known.values()))
        for n, q in known.items():
            self.tracker.park(n, q[0], q[1])

    def park_all(self, names):
        """Heading and parking, robot by robot, in an order where nobody's park spot is taken by another robot.
        A robot standing on someone else's spot when all spots are taken goes to the middle of the board first."""
        todo = [n for n in names if n in self.st["positions"] and n not in self.unavailable]
        deferred = defaultdict(int)
        passes = 0
        while todo:
            passes += 1
            if passes > 3 * len(names) + 2:
                raise SystemExit("Parking went round in circles (%s still not parked). Put the robots anywhere open on "
                                 "the board, 30 cm apart, and start again." % ", ".join(todo))
            def blocked(n):
                i = self.park_index(n)
                px_, py_ = self.board.park(i)[:2]
                sx_, sy_ = self.board.stage(i)
                # another robot on its spot, or right on the point it backs in from (2026-10-22: a looser test for
                # the backing-in point; 24 cm round both dead-locked robots put down 30 cm from the walls)
                return [m for m in todo if m != n and (
                    math.hypot(self.st["positions"][m][0] - px_, self.st["positions"][m][1] - py_) < PL.PARK_CLEAR_CM + 6
                    or math.hypot(self.st["positions"][m][0] - sx_, self.st["positions"][m][1] - sy_) < PL.PARK_CLEAR_CM)]
            def path_ok(n):
                q0 = self.st["positions"][n]
                return self.safe_path(n, (q0[0], q0[1]), self.board.stage(self.park_index(n))) is not None

            def obstructs(m):
                # how many other robots' ways (to their backing-in point, and their spots) m stands in
                qm = self.st["positions"][m]
                k = 0
                for n2 in todo:
                    if n2 == m:
                        continue
                    a = self.st["positions"][n2]
                    for b in (self.board.stage(self.park_index(n2)), self.board.park(self.park_index(n2))[:2]):
                        dx, dy = b[0] - a[0], b[1] - a[1]
                        L = math.hypot(dx, dy) or 1e-6
                        t = max(0.0, min(L, ((qm[0] - a[0]) * dx + (qm[1] - a[1]) * dy) / L))
                        if math.hypot(a[0] + dx / L * t - qm[0], a[1] + dy / L * t - qm[1]) < PL.PARK_CLEAR_CM + 4:
                            k += 1
                return k
            # park first a robot whose spot is free AND whose way there is clear (2026-10-22: checking only the
            # spot dead-locked three robots put down in the corners); none: the one most in the others' way moves
            # to a free holding spot first
            free = [n for n in todo if not blocked(n) and path_ok(n)]
            if not free:
                todo.sort(key=lambda m: -obstructs(m))
            n = free[0] if free else todo[0]
            self.active = n
            self.phase = "finding"
            q = self.st["positions"][n]
            p = self.acquire_heading(n, (q[0], q[1]))
            if p is None:
                if not self.help_from_nick(n, "the camera lost it while measuring which way it faces"):
                    todo.remove(n)
                    continue
            elif not free:
                mid = self.holding_spot(n)
                self.say("  Every park spot has another robot near it: moving %s out of the way first, to (%.0f, %.0f) cm."
                         % (n, mid[0], mid[1]))
                ok, why = self.drive_to(n, mid, max_leg=PARK_LEG_CM)
                self.end_moves(n)
                if not ok:
                    if not self.help_from_nick(n, why):
                        todo.remove(n)
                    continue
                p = self.settle(min_s=0.3)
                if p:
                    self.st["positions"][n] = [p[0], p[1], self.heads.get(n, [None])[0]]
                    self.tracker.park(n, p[0], p[1])
                continue
            self.set_corrections(n, self.move_corrections(n))
            ok, why = self.park(n)
            if not ok and "no clear way" in why and deferred[n] < 2 and len(todo) > 1:
                # another robot not parked yet is in the way: park the others first, then try again
                deferred[n] += 1
                self.end_moves(n)
                p = self.settle(min_s=0.3)
                if p:
                    self.st["positions"][n] = [p[0], p[1], self.heads.get(n, [None])[0]]
                    self.tracker.park(n, p[0], p[1])
                self.say("  %s waits: another robot is in its way; parking the others first." % n)
                todo.remove(n)
                todo.append(n)
                continue
            if not ok and not self.help_from_nick(n, why):
                todo.remove(n)
                continue
            p = self.settle()
            if p:
                self.say("  %s parked at (%.1f, %.1f) cm" % (n, p[0], p[1]))
            todo.remove(n)
        self.st.save()

    def holding_spot(self, robot):
        """Where a robot can wait out of everyone's way while the others park (2026-10-22: robots put down in three
        corners dead-locked; the middle was used without checking it was free): the point on the board furthest
        from every other robot, every park spot and every backing-in point, and well in from the edges."""
        others = [tuple(q[:2]) for n, q in self.st["positions"].items() if n != robot and q]
        spots = []
        for i in range(len(self.st["robots"])):
            spots.append(tuple(self.board.park(i)[:2]))
            spots.append(tuple(self.board.stage(i)))
        best = None
        for x in np.arange(self.board.x0 + 25, self.board.x1 - 24, 4.0):
            for y in np.arange(self.board.y0 + 25, self.board.y1 - 24, 4.0):
                d_r = min([math.hypot(x - o[0], y - o[1]) for o in others] or [99.0])
                d_s = min([math.hypot(x - q[0], y - q[1]) for q in spots] or [99.0])
                score = min(d_r, 1.5 * d_s)
                if best is None or score > best[0]:
                    best = (score, (float(x), float(y)))
        return best[1]

    def acquire_heading(self, robot, p):
        """A robot found by identify(): follow it from p and measure its heading by a short drive (forward;
        blocked: a quarter turn and again). Returns its position or None."""
        self.fresh_id[robot] = True
        self.tracker.robot_at(p[0], p[1], scale=ID_UNKNOWN_SCALE)
        self.tracker.follow(robot, p[0], p[1])
        self.history.clear()
        self.tracker.hold(HOLD_CM)
        self.heads[robot] = [None, False]
        self.moves.pop(robot, None)
        # the tracker settles from the find's rough spot onto the robot (dry run 19:10, at the bottom edge: 13.1 ->
        # 7.5 cm in a second); that settling is not the robot moving: wait for it, then drop those pictures, so the
        # watchdog does not take it for a dash at the edge and the first fix is the settled spot
        if self.settle(t_from=time.time() + 0.3, min_s=0.5, timeout=2.5) is not None and self.history is not None:
            self.history.clear()
        if not self.begin_moves(robot, "finding its heading"):
            return None
        q_ = self.pos(robot) and self.pos(robot)[1:3] or p
        rm_ = self.signed_room(q_)
        room = None if rm_ is None else rm_[0]
        if room is not None and not self.board.on_board(q_[0], q_[1], margin=HEADING_CRAWL_ROOM_CM):
            # the camera puts it off the board, or near a side the camera may have wrong (dry run 19:31 after a poor
            # explore fit: Center seen at x = 135, past the board, really 3 cm from the open edge; three 15 cm heading
            # drives took it there): the crawl too
            room = min(room, 0.0)
        if room is not None and room < HEADING_CRAWL_ROOM_CM:
            # fix 2 (15:45: Left's 4 cm heading drive from 11 cm went forward at the edge and put its nose at the
            # drop): near an open edge no blind drive, a crawl (Nick's choice)
            return self.heading_crawl(robot)
        for attempt in range(4):
            # look before driving (2026-10-22 evening: Left, put down against the left board facing it, drove its
            # heading leg into the board; the wheels spun "10 cm", it stayed put, and its outline merged with the
            # board's dark edge, three times over): with something close ahead, a quarter turn first
            f = self.front_now(robot)
            if f is not None and f < HEADING_CLEAR_CM:
                self.say("    %s: something %.0f cm in front; turning before its heading drive" % (robot, f), quiet=True)
                self.leg(robot, ("turn", 90.0))
                continue
            # 15 cm where there is room (Nick, 2026-10-23: a 1 cm position error is 6 deg over 10 cm, 4 deg over 15
            # cm); 10 cm near an open edge, whose direction from it is unknown (chosen; DOE: tests-to-run)
            q = self.pos(robot) or p
            far = self.board.edge_room(q[0], q[1])[0] > HEADING_LONG_ROOM_CM and (f is None or f > 15.0 + HEADING_CLEAR_CM)
            # which way it faces is not known yet, so this short drive forward may head at an edge: the firmware's
            # edge guard stops it there (forwards); the virtual border lets it (Pi, 2026-10-24 19:35: Right's heading
            # drive held the session at 13 cm), and a moment after
            self.seek.add(robot)
            self.fw_seek(robot, True)
            try:
                p0, p1, words = self.leg(robot, ("go", 15.0 if far else 10.0))
            finally:
                self.fw_seek(robot, False)
                self.seek.discard(robot)
                self.seek_end[robot] = time.time() + 0.3     # + the picture's age
            if p0 and p1 and math.hypot(p1[0] - p0[0], p1[1] - p0[1]) >= ST.MIN_HEADING_CM:
                self.heads[robot] = [ST.bearing(p0, p1), True]
                self.say("  %s faces %.0f deg" % (robot, self.heads[robot][0]))
                self.heading_push(robot, self.heads[robot][0])    # heading fix 3: tell the robot at once
                return p1
            if "disagree" in words and self.disagree[robot] >= 2:
                return None
            self.leg(robot, ("turn", 90.0))
        return None

    def heading_crawl(self, robot):
        """Fix 2 (Nick, 2026-10-25 18:46): which way a robot faces, found near an open edge without a blind drive:
        it crawls forward HEADING_CRAWL_CM at HEADING_CRAWL_CMS with its edge guard on (and the camera border
        letting it, as explore's crawls), at most twice until the camera has seen it move ST.MIN_HEADING_CM. If its
        edge guard stops it, it faces the edge: it backs straight out HEADING_CRAWL_BACK_CM (the way it came is
        clear; the watchdog lets a back-out run), and the camera measures the heading on the way back. Sets
        heads[robot] (measured) and returns its position, or None (heading still unknown). Logged
        (heading_crawls)."""
        # a still position the tracker has had time to lock on (a find's first position is rough: dry run 19:03, the
        # crawl measured end to end from it came out 44 deg off)
        p_start = self.settle(t_from=time.time() + 0.3, min_s=0.5, timeout=2.5) or self.settle(min_s=0.3, timeout=2.0)
        if p_start is None:
            return None
        rm_ = self.signed_room(p_start)
        room0 = None if rm_ is None else rm_[0]
        if room0 is not None and room0 < CRAWL_MIN_ROOM_CM:
            self.say("    %s: only %.0f cm from an open edge: no crawl (its edge sensors may be over it)" % (robot, room0))
            self.st.setdefault("heading_crawls", []).append(dict(t=round(time.time(), 1), robot=robot,
                                                                 room_cm=round(room0, 1), refused=True))
            return None
        self.say("    %s: %s from an open edge, which way it faces not known: a %.0f cm crawl with its edge sensors "
                 "watching" % (robot, "%.0f cm" % room0 if room0 is not None else "near", HEADING_CRAWL_CM),
                 quiet=True)
        old_cms = self.speed_now.get(robot)
        self.explore_speed(robot, HEADING_CRAWL_CMS)
        edge, words, p_end = False, "", p_start
        t_crawl = time.time()
        wheels = [0.0, 0]           # cm its wheels drove on the crawl legs, legs counted
        self.seek.add(robot)
        self.fw_seek(robot, True)
        try:
            for step in range(2):
                _a, p_b, words = self.leg(robot, ("go", HEADING_CRAWL_CM))
                if p_b is not None:
                    p_end = p_b
                self.add_wheels(robot, wheels)
                if "plywood edge" in (words or ""):
                    edge = True
                    break
                if not (words or "").startswith(("done", "dup")) or \
                        self.track_heading(t_crawl, p_start, p_end, self.wheels_of(wheels))[0] is not None:
                    break                     # (a heading already: no second step)
        finally:
            self.fw_seek(robot, False)
            self.seek.discard(robot)
            self.seek_end[robot] = time.time() + 0.3
            self.explore_speed(robot, old_cms or EXPLORE_FAST_CMS)
        rec = dict(t=round(time.time(), 1), robot=robot, room_cm=None if room0 is None else round(room0, 1),
                   start=[round(v, 1) for v in p_start], end=[round(v, 1) for v in p_end], edge=edge, words=words)
        h = None
        if edge:
            t_back = time.time()
            q0, q1, w2 = self.edge_back_out(robot, HEADING_CRAWL_BACK_CM)
            rec["back"] = w2
            wb = [0.0, 0]
            self.add_wheels(robot, wb)
            if q0 is not None and q1 is not None:
                hb, moved = self.track_heading(t_back, q0, q1, self.wheels_of(wb))
                if hb is not None and moved >= CRAWL_HEADING_MIN_CM:
                    h = ST.wrap180(hb + 180.0)                  # it backed: it faces the other way
            if q1 is not None:
                p_end = q1
        else:
            hf, moved = self.track_heading(t_crawl, p_start, p_end, self.wheels_of(wheels))
            if hf is not None and moved >= CRAWL_HEADING_MIN_CM:
                h = hf
            elif "obstacle" in (words or "") and (room0 is None or room0 >= HEADING_CRAWL_ROOM_CM):
                # 2026-10-26 09:55 session (10:49, Left in the blocked corner facing a board): its front stop refused
                # every crawl, which read as "lost". Something is in front: crawl backwards instead, the same way
                # (away from a board, not near an open edge: the back has no edge guard)
                self.say("    %s: something in front (its front stop): crawling backwards instead" % robot, quiet=True)
                t_back = time.time()
                old2 = self.speed_now.get(robot)
                self.explore_speed(robot, HEADING_CRAWL_CMS)
                try:
                    q0, q1, w2 = self.leg(robot, ("go", -HEADING_CRAWL_CM))
                finally:
                    self.explore_speed(robot, old2 or EXPLORE_FAST_CMS)
                wb = [0.0, 0]
                self.add_wheels(robot, wb)
                rec["backwards"] = w2
                if q0 is not None and q1 is not None:
                    hb, moved = self.track_heading(t_back, q0, q1, self.wheels_of(wb))
                    if hb is not None and moved >= CRAWL_HEADING_MIN_CM:
                        h = ST.wrap180(hb + 180.0)          # it backed: it faces the other way
                    p_end = q1
        rec["heading"] = None if h is None else round(h, 1)
        self.st.setdefault("heading_crawls", []).append(rec)
        if h is None:
            self.say("    %s: the crawl did not show which way it faces (%s)" % (robot, words), quiet=True)
            return None
        self.heads[robot] = [h, True]
        self.say("  %s faces %.0f deg (crawl%s)" % (robot, h, ", then backed out from the edge" if edge else ""))
        self.heading_push(robot, h)
        return p_end

    LOCKED_SOURCES = ("lock", "both")

    def track_heading(self, t0, p0, p1, wheels_cm=None):
        """(direction of travel deg or None, cm moved) since t0: the straight line through the camera's pictures
        of it where the tracker was locked on (ST.heading_from_track, as a run's heading), else p0 to p1 when there
        are too few pictures. wheels_cm: what its wheels drove meanwhile; a camera that saw under
        CRAWL_CAMERA_SHARE of it was not following the robot: no heading (dry run 19:18 at the bottom edge: the
        lock lost, the detector's positions jumping 3 cm, a fitted heading 165 deg off)."""
        hist = getattr(self, "history", None)
        smp = [(q[0], q[1], q[2]) for q in (hist.since(t0) if hist is not None else [])]
        sl = getattr(self, "src_log", None)
        if sl:
            locked = {t for t, src in list(sl) if src in self.LOCKED_SOURCES and t >= t0 - 0.01}
            n_all = len(smp)
            smp = [q for q in smp if q[0] in locked]
            if n_all and len(smp) < CRAWL_LOCKED_SHARE * n_all:
                return None, 0.0          # the lock came and went: its direction is not to be trusted
        moved = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) if (p0 is not None and p1 is not None) else 0.0
        if len(smp) >= 5:
            moved = math.hypot(smp[-1][1] - smp[0][1], smp[-1][2] - smp[0][2])
            if wheels_cm is not None and wheels_cm > 1.0 and moved < CRAWL_CAMERA_SHARE * wheels_cm:
                return None, moved
            h = ST.heading_from_track(smp, min_cm=CRAWL_HEADING_MIN_CM, max_rms_cm=0.6)
            if h is not None:
                return h, moved
            return None, moved
        if wheels_cm is not None and wheels_cm > 1.0 and moved < CRAWL_CAMERA_SHARE * wheels_cm:
            return None, moved
        if moved >= ST.MIN_HEADING_CM:
            return ST.bearing(p0, p1), moved
        return None, moved

    def add_wheels(self, robot, acc):
        """Add what the robot's wheels say its last leg drove (cm) to acc = [cm, legs]."""
        lis = getattr(self, "listener", None)
        if lis is None:
            return
        w = self.wheels_moved(robot, self.last_lid.get(robot))
        if w is not None:
            acc[0] += w
            acc[1] += 1

    @staticmethod
    def wheels_of(acc):
        return acc[0] if acc[1] else None

    def edge_back_out(self, robot, cm):
        """Fix 4: back straight out cm right after the robot's own edge guard stopped it. The watchdog lets it run
        (edge_stop_t), and no room check is made behind it (the way it came). Returns leg()'s (p0, p1, words)."""
        self.__dict__.setdefault("edge_stop_t", {}).setdefault(robot, time.time())
        self.st.setdefault("edge_back_outs", []).append([round(time.time(), 1), robot, cm])
        self.say("    %s: its edge sensors stopped it: backing straight out %.0f cm" % (robot, cm), quiet=True)
        return self.leg(robot, ("go", -abs(cm)), backout=True)

    def front_now(self, robot):
        """The front sensor's reading now (cm), by a tiny turn leg (firmware 2026-10-22b reports front= with every
        finished leg); None if there is no reading."""
        state, info = self.send_leg(robot, "turn 2.0", 6.0)
        m = re.search(r"front=([0-9.]+)", info or "")
        if state not in ("done", "dup") or not m or float(m.group(1)) <= 0:
            return None
        return float(m.group(1))

    def map_board(self, robot, n_rays=16):
        """Robot perimeter mapping (e02_map.py): from the middle of the board, rays out in n_rays directions; each
        ends where the robot's own sensors stop it (front stop at a board, edge guard at an open edge); then
        straight back. The camera only watches (its watchdog lets it reach the edge itself: that is what is
        measured; it still stops a robot whose centre would cross the edge). Returns the sides found, or None."""
        st = self.st
        if not self.activate(robot):
            self.say("Mapping: the camera can't see %s at its spot; no mapping this time." % robot)
            return None
        self.say("=== Mapping: %s feels out the board (%d rays from the middle; about %d minutes) ===" % (
            robot, n_rays, max(1, round(n_rays * 22 / 60))))
        self.mark("e02 map start %s" % robot)
        self.phase = "mapping"
        self.order = "map"
        self.set_corrections(robot, self.move_corrections(robot))
        mid = ((self.board.x0 + self.board.x1) / 2, (self.board.y0 + self.board.y1) / 2)
        ok, why = self.drive_to(robot, mid, max_leg=PARK_LEG_CM, arrive_cm=4.0)
        if not ok:
            self.say("Mapping: could not get %s to the middle (%s); no mapping this time." % (robot, why))
            self.park(robot)
            return None
        rays = []
        self.watch_edge_cm = 0.0
        try:
            others = [(n, tuple(q[:2])) for n, q in st["positions"].items() if n != robot and q]
            (_, cx), (_, cy) = self.board.open_edges()
            self.disagree[robot] = 0
            for k, h in enumerate(MAP.ray_headings(n_rays)):
                if MAP.toward_corner(mid, h, (cx, cy)):
                    self.say("  ray %2d, %4.0f deg: skipped (toward the corner of the two open edges)" % (k + 1, h))
                    rays.append(dict(k=k, heading=round(h, 1), skipped="open corner", kind=None))
                    continue
                if self.disagree[robot] >= 2:
                    self.say("  the camera and %s's wheels disagreed twice: mapping ends here" % robot)
                    break
                blocker = MAP.ray_blocked(mid, h, others)
                if blocker:
                    self.say("  ray %2d, %4.0f deg: skipped (%s is parked that way)" % (k + 1, h, blocker))
                    rays.append(dict(k=k, heading=round(h, 1), skipped=blocker, kind=None))
                    continue
                p = self.settle(min_s=0.3)
                if p is None:
                    break
                if math.hypot(p[0] - mid[0], p[1] - mid[1]) > 8.0:
                    ok, why = self.drive_to(robot, mid, max_leg=PARK_LEG_CM, arrive_cm=4.0)
                    if not ok:
                        self.say("  could not get back to the middle (%s): mapping ends here" % why)
                        break
                hd = self.heads.get(robot, [None])[0]
                if hd is None:
                    self.leg(robot, ("go", 5.0))
                    hd = self.heads.get(robot, [None])[0]
                    if hd is None:
                        break
                turn = ST.wrap180(h - hd)
                if abs(turn) >= ST.MIN_TURN_DEG:
                    self.leg(robot, ("turn", turn))
                self.heads[robot] = [h, False]
                # heading fix 4: h is the COMMANDED heading after a turn: its own gyro saw the real turn
                self.fix_now(robot, position_only=self.own_pose(robot) is not None)
                p0, p1, words = self.leg(robot, ("go", MAP.RAY_CM))
                lid = self.main_lid.get(robot)
                fin = self.listener.leg_final(robot, "-", lid) or {}
                kind = MAP.stop_kind(fin.get("state"), fin.get("info"))
                rx, ry, rth = num(fin.get("x_cm")), num(fin.get("y_cm")), num(fin.get("th_deg"))
                ray = dict(k=k, heading=round(h, 1), state=fin.get("state"), info=fin.get("info"), kind=kind,
                           words=words, cam_start=p0, cam_end=p1,
                           robot_end=[rx, ry, rth] if rx is not None else None)
                if p0 and p1 and math.hypot(p1[0] - p0[0], p1[1] - p0[1]) >= ST.MIN_HEADING_CM:
                    self.heads[robot] = [ST.bearing(p0, p1), True]
                if kind and rx is not None:
                    ray["point"] = MAP.contact_point(rx, ry, rth, kind)
                    if p1:
                        hh = self.heads[robot][0]
                        ray["point_cam"] = MAP.contact_point(p1[0], p1[1], rth if hh is None else hh, kind)
                rays.append(ray)
                self.say("  ray %2d, %4.0f deg: %s%s" % (k + 1, h, {"edge": "open edge", "board": "board"}.get(kind, "nothing met (%s)" % (words or fin.get("state"))),
                                                       (" at (%.1f, %.1f) cm" % tuple(ray["point"])) if ray.get("point") else ""))
                # straight back: off the edge first (a turn on the spot puts a wheel over an edge it is this close
                # to: firmware edge.h), then turn round and forward (the guards look ahead; backing up is blind)
                if p1 is None and words.startswith("camera and wheels disagree"):
                    self.say("  the camera can't find %s again after this ray: mapping ends here" % robot)
                    break
                back = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) if (p0 and p1) else None
                q = p1 or p0
                if kind == "edge" or (q and self.board.edge_room(q[0], q[1])[0] < EDGE_TURN_CM + 3):
                    self.leg(robot, ("go", -7.0))
                    back = (back - 7.0) if back else None
                self.leg(robot, ("turn", 180.0))
                if self.heads.get(robot, [None])[0] is not None:
                    self.heads[robot] = [ST.wrap180(self.heads[robot][0] + 180.0), False]
                if back and back > 2.0:
                    self.leg(robot, ("go", back))
        finally:
            self.watch_edge_cm = None
        others = [tuple(q[:2]) for n, q in st["positions"].items() if n != robot and q]
        pts = [(r["point"][0], r["point"][1], r["kind"]) for r in rays if r.get("point")]
        sides = MAP.fit_sides(pts, self.board, others=others)
        for line in MAP.summary_words(sides, self.board):
            self.say(line)
        safe, risky = MAP.layout_check(sides, self.board)
        for line in safe + risky:
            self.say(line)
        st["setup"]["layout_check"] = dict(safe=safe, risky=risky)
        # the blocked boards: felt vs the picture
        felt = {}
        for key, sd in sides.items():
            if sd["kind"] != "board":
                continue
            if sd["axis"] == "y":
                felt["x"] = sd["value"]          # the x-board is the line y = const
            else:
                felt["y"] = sd["value"]
        img_lines = st["setup"].get("board_lines_image", {})
        for which, v in felt.items():
            pic = img_lines.get(which, {}).get("value")
            if pic is not None:
                self.say("  %s-board: felt at %.1f cm, the picture says %.1f cm (%+.1f cm)" % (which, v, pic, v - pic))
        st["setup"]["map"] = dict(robot=robot, rays=rays, sides=sides, felt_boards=felt, t=round(time.time(), 1))
        st.save()
        MAP.save(os.path.join(self.dir, "map.json"), robot, rays, sides, self.board)
        try:
            self.draw_map(rays, sides)
        except Exception as e:
            self.say("  (map picture failed: %s)" % e, quiet=True)
        self.mark("e02 map end %s" % robot)
        if risky:
            st.save()
            raise SystemExit("The robots felt an open edge where the setup took a board (above). A find drives into "
                             "the boards, so it would go over that edge. Start the runner again (without --resume) and answer the two "
                             "board questions by what is really there.")
        ok, why = self.park(robot)
        if not ok:
            self.help_from_nick(robot, why)
        return sides

    def draw_map(self, rays, sides):
        import cv2
        img = self.feed.last_img
        if img is None:
            return
        v = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        for r in rays:
            p0, pt = r.get("cam_start"), r.get("point")
            if p0 and pt:
                a = self.cam.project((p0[0], p0[1], 0.0))
                b = self.cam.project((pt[0], pt[1], 0.0))
                col = (0, 0, 255) if r["kind"] == "edge" else (255, 160, 0)
                cv2.line(v, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), col, 1)
                cv2.circle(v, (int(b[0]), int(b[1])), 3, col, -1)
        path = os.path.join(self.dir, "check_map.png")
        cv2.imwrite(path, v)
        self.say("  The map, drawn on the camera picture: %s (red: open edges, blue: boards)" % path)

    # --- the brick demo (Nick, 2026-10-22; e02_bricks.py) ---------------------------------------------
    def brick_demo(self):
        """Nick puts bricks (or any objects) on the board; the camera finds what changed, plans a path round them,
        and one robot crosses three times on the same path (the ladder, BR.LADDER): (1) factory settings, wheels
        and gyro only; (2) wheels and gyro with the learned corrections, the camera only watching; (3) the same
        path with camera fixes between legs. --tight-gap: the room kept round the objects comes from the robot's
        measured precision this session, and the path goes straight through the narrowest gap it can safely make.
        Then the objects come off and it parks. The edge guard and front stop stay on (and the camera watchdog
        guards the objects): a crossing they stop is recorded as failed, and the robot is backed off before the
        next. Returns the result dict, or None."""
        import e02_bricks as BR
        st = self.st
        names = [n for n in st["robots"] if n not in self.unavailable and st["positions"].get(n)]
        if not names:
            self.say("Brick demo: no robot is parked on the board, so there is nothing to show.")
            return None
        tight = bool(getattr(self.a, "tight_gap", False))
        k = float(getattr(self.a, "gap_k", BR.GAP_K) or BR.GAP_K)
        precs = {n: {m: BR.gap_precision(st.get("schedule") or [], n, m, k=k) for m in BR.LADDER} for n in names}
        print("")
        self.say("=== Brick demo: a robot finds its way %s ===" % (
            "through a tight gap between objects you put down" if tight else "round objects you put down"))
        self.phase, self.order, self.active = "bricks", "bricks", None
        need = min(precs[n]["camera"]["gap_cm"] for n in names)
        if tight:
            self.say("How precise each robot is this session (its camera-measured errors), and so the narrowest gap "
                     "it can safely drive through:")
            for n in names:
                for line in BR.precision_words(n, precs[n]):
                    self.say(line)
        self.say("Taking a picture of the board as it is now (keep your hands out of the picture) ...")
        self.fresh_floor()
        time.sleep(0.5)
        floor = BR.floor_snapshot(self.tracker)
        if floor is None:
            self.say("Brick demo: the camera has no picture of the floor yet; no demo this time.")
            return None
        flag = os.path.join(self.dir, "sim", "BRICKS_ON")
        if self.a.dry_run:
            open(flag, "w").close()                 # the simulator puts the bricks down now
            time.sleep(1.0)
        self.mark("e02 bricks place")
        ch = found = None
        while True:
            print("\n" + "-" * 78)
            self.say("Now put the objects down (bricks, or anything to hand: a shoe, a mug, a pen, a cable):")
            if tight:
                self.say("1. Put two objects on the open part of the board with a gap of about %.0f cm (%.0f in) "
                         "between them: at least 30 cm (1 ft) from every robot and at least 20 cm (8 in) from the "
                         "open edges, with clear board on both sides of the gap." % (need + 4.0, (need + 4.0) / 2.54))
                self.say("   (The robot needs %.0f cm as the camera sees it; the camera sees each object about 2 cm "
                         "bigger than it is, hence the extra. Anything wider than %.0f cm is still fine, just less "
                         "impressive.)" % (math.ceil(need), need + 10.0))
            else:
                self.say("1. Put them anywhere on the open part of the board: at least 30 cm (1 ft) from every robot "
                         "and at least 20 cm (8 in) from the open edges.")
                self.say("   Leave at least one gap about 25 cm (10 in) wide between them, so a robot can get through.")
            self.say("2. Don't touch or move the robots.")
            self.say("3. Step back so your hands and your shadow are out of the camera picture.")
            print("-" * 78)
            self.ask("4. Then press Enter here: ", "")
            frames = self.still_frames()
            robots = {n: tuple(st["positions"][n][:2]) for n in names}
            found = BR.find_bricks(self.cam, frames, floor, self.board_rect(), robots=list(robots.values()))
            for line in BR.brick_words(found["bricks"], found):
                self.say(line)
            if not found["bricks"] and not self.a.auto:
                ans = self.ask("Check the objects are inside the board's outline on the live view, then press Enter "
                               "to look again (or type go and press Enter to cross with no objects): ", "")
                if ans.lower() != "go" or tight:
                    continue
            if tight:
                ch = self._choose_tight_gap(found, robots, precs, need)
                if ch is None:
                    if self.a.auto:
                        self._bricks_off(flag)
                        return None
                    self.say("  Move the objects so the gap is about %.0f cm (%.0f in), step back, and press Enter."
                             % (need + 4.0, (need + 4.0) / 2.54))
                    continue
            else:
                ch = BR.choose_crossing(found["occ"], found["grid"], self.board, robots)
            if ch is None:
                self.say("  There is no way across: the objects leave no gap a robot can get through (it needs about "
                         "25 cm, 10 in), or they are too close to the robots.")
                if self.a.auto:
                    self._bricks_off(flag)
                    return None
                self.say("  Move one object to open a gap, step back, and press Enter.")
                continue
            for line in BR.plan_words(ch):
                self.say(line)
            self.bricks_view = dict(bricks=found["bricks"], room_cm=round(ch["space"]["r_cm"], 1),
                                    path=list(ch["path"]["points"]), start=ch["start"], goal=ch["goal"], tracks={})
            pic = self.save_bricks_picture()
            self.say("  The plan is drawn on the live view%s: red outlines are the objects, the blue line is the path, "
                     "the white dot the start and the white ring the goal%s." % (
                         " (http://localhost:%d)" % self.a.view_port if self.a.view_port else "",
                         (" (also saved as %s)" % pic) if pic else ""))
            ans = self.ask("Press Enter to start the three crossings (or type r and press Enter to move the objects "
                           "and look again): ", "")
            if ans.lower().startswith("r"):
                self.bricks_view = None
                continue
            break
        self.mark("e02 bricks found %d" % len(found["bricks"]))
        self.fresh_floor()                 # the bricks are part of the floor now: the tracker follows only the robot
        robot = ch["robot"]
        corr = self.move_corrections(robot)
        source = "learned from the camera in this session's calibration runs" if self.fitted(robot) else \
            "the starting guesses (this session has no calibration runs yet%s)" % (
                "; the gyro creep measured in the naming check" if ((st.get("naming_check") or {}).get(
                    "gyro_raw_drift_dps") or {}).get(robot) is not None else "")
        self.say("  %s crosses. Its corrections: %s (%s)." % (robot, FIT.describe(corr), source))
        out = dict(t=round(time.time(), 1), robot=robot, bricks=found["bricks"], start=ch["start"], goal=ch["goal"],
                   path=ch["path"], approach=ch["approach"]["points"], words=ch["words"],
                   straight_blocked_cm=ch["straight_blocked_cm"], corrections=corr, corrections_source=source,
                   factory=dict(gyro_scale=1.0, gyro_drift_dps=0.0, wheel_scale=1.0),
                   room_cm=round(ch["space"]["r_cm"], 1), settings=dict(margin_cm=BR.MARGIN_CM,
                                                                        open_edge_cm=BR.OPEN_EDGE_CLEAR_CM,
                                                                        board_cm=BR.BOARD_CLEAR_CM,
                                                                        robot_cm=BR.ROBOT_CLEAR_CM),
                   left_out=dict(specks=len(found.get("specks") or []), rim=found.get("rim", 0)),
                   tight_gap=bool(tight), precision=precs.get(robot), crossings=[])
        if ch.get("gap"):
            out.update(gap=ch["gap"], clearance_cm=ch["clearance_cm"], gap_k=k)
        others = [tuple(q[:2]) for n, q in st["positions"].items() if n != robot and q]
        space = BR.free_space(found["occ"], found["grid"], self.board, others=others)
        if ch.get("gap"):              # to and from the start: also back through the gap along its line
            space = BR.corridor_space(space, BR.gap_space(found["occ"], found["grid"], self.board, others,
                                                          ch["clearance_cm"]), ch["path"]["points"])
        self.bricks_guard = space          # the camera watchdog guards the objects from now until they are off
        try:
            if not self.activate(robot):
                self.say("  The camera can't see %s at its spot: no crossings this time." % robot)
                return self._bricks_end(robot, out, flag, space)
            self.set_corrections(robot, corr)
            first = ST.bearing(ch["path"]["points"][0], ch["path"]["points"][1])
            if ch.get("gap"):
                # a straight run through the gap would test only the wheels; starting side-on puts a 90 deg turn
                # first, where an uncorrected gyro shows (about 7 deg off at factory settings: 7 cm sideways at 60 cm)
                first = ST.wrap180(first + 90.0)
                out["start_turn_deg"] = -90.0
            for n, method in enumerate(BR.LADDER, 1):
                self.set_corrections(robot, corr)            # getting to the start: always its learned corrections
                self.say("  Getting %s to the start (the camera steers this part; it is not measured) ..." % robot)
                ok, why = self.bricks_to(robot, space, ch["start"], final_heading=first)
                if not ok:
                    self.say("  %s could not get to the start: %s." % (robot, why))
                    if not self.help_from_nick(robot, "it could not get to the brick demo's start: " + why, park=False):
                        break
                    self.activate(robot)
                    ok, why = self.bricks_to(robot, space, ch["start"], final_heading=first)
                    if not ok:
                        self.say("  Still no way to the start (%s): the demo stops here." % why)
                        break
                if method == "factory":
                    self.begin_moves(robot)                  # (already moving: no change) then factory values
                    self.set_corrections(robot, None)        # the U condition's values: gyro 1, drift 0, wheels 1
                    time.sleep(0.3)
                c = self.brick_crossing(robot, ch, space, n, method)
                self.set_corrections(robot, corr)
                c["corrections"] = None if method == "factory" else corr
                out["crossings"].append(c)
                for line in BR.crossing_words(c):
                    self.say(line)
                BR.save(os.path.join(self.dir, "bricks.json"), out)
                if c.get("safety"):
                    if not self.bricks_back_off(robot, space, c):
                        break
                    moved = self.objects_moved(floor, found, names, robot)
                    if moved:
                        c["objects_moved_cm2"] = moved
                        self.say("  An object was pushed (about %.0f cm2 of the board looks different from the plan)."
                                 % moved)
                        if not self.a.auto:
                            self.ask("  Put it back on its red outline on the live view, step back, and press Enter: ",
                                     "")
                            self.fresh_floor()
        finally:
            self.end_moves(robot)
        return self._bricks_end(robot, out, flag, space)

    def _choose_tight_gap(self, found, robots, precs, need):
        """Tight-gap mode: the gaps between the objects, which one to cross (the narrowest a robot can safely make,
        or the one Nick types), and the crossing straight through it. Returns a crossing dict or None."""
        import e02_bricks as BR
        gaps = BR.find_gaps(found["occ"], found["grid"], found["bricks"])
        best = min(robots, key=lambda n: precs[n]["camera"]["gap_cm"])
        for line in BR.gap_words(gaps, need, best):
            self.say(line)
        ok = [i for i, g in enumerate(gaps) if g["width_cm"] + 1e-6 >= need]
        if not ok:
            self.say("  No gap is wide enough: %s needs %.0f cm (as the camera sees it)." % (best, math.ceil(need)))
            return None
        pick = ok[0]                                    # the narrowest safe gap: the most impressive
        if not self.a.auto and len(gaps) > 1:
            ans = self.ask("Press Enter to cross gap %d (the narrowest it can safely make), or type another gap's "
                           "number and press Enter: " % (pick + 1), "")
            if ans.strip().isdigit() and 1 <= int(ans) <= len(gaps):
                pick = int(ans) - 1
                if pick not in ok:
                    self.say("  Gap %d is too narrow to cross safely; crossing gap %d." % (pick + 1, ok[0] + 1))
                    pick = ok[0]
        clear = {n: precs[n]["camera"]["clearance_cm"] for n in robots}
        for i in [pick] + [j for j in ok if j != pick]:
            ch = BR.choose_gap_crossing(found["occ"], found["grid"], self.board, robots, gaps[i], clear)
            if ch is not None:
                if i != pick:
                    self.say("  Gap %d has no clear board on both sides (or no robot can get there): crossing gap %d."
                             % (pick + 1, i + 1))
                self.say("  %s crosses gap %d: %.1f cm wide as the camera sees it; it needs %.1f cm." % (
                    ch["robot"], i + 1, gaps[i]["width_cm"], 2.0 * ch["clearance_cm"]))
                return ch
        self.say("  No safe gap has clear board on both sides for a straight run through it (about 25 cm each side), "
                 "or no robot can get to it.")
        return None

    def bricks_back_off(self, robot, space, c):
        """After a crossing the safety stopped (front stop, edge guard, watchdog): back the robot straight up, away
        from the objects, so the next crossing starts clean. Returns True when it is free to go on."""
        import e02_bricks as BR
        p = self.settle(min_s=0.4)
        h = c.get("end_heading")
        if p is None or h is None:
            return self.help_from_nick(robot, "the safety stopped it next to an object and the camera can't tell which "
                                              "way it faces", park=False)
        self.heads[robot] = [h, True]
        d = BR.back_off_cm(space, p, h, want_cm=12.0)
        if d < 2.0:
            self.say("  %s is already clear of the objects behind it; it goes on from here." % robot)
            return True
        self.say("  Backing %s up %.0f cm, straight away from what stopped it, before the next crossing ..." % (robot, d))
        self.watch_stop = None
        p0, p1, words = self.leg(robot, ("go", -round(d, 1)))
        c["backed_off_cm"] = round(d, 1)
        if not (words or "").lower().startswith(("done", "dup")):
            self.say("  It could not back up (%s)." % words)
            return self.help_from_nick(robot, "it could not back away from an object: " + words, park=False)
        return True

    def objects_moved(self, floor, found, names, robot):
        """After a stopped crossing: did the robot push an object? The objects again, against the same floor
        picture; returns the board area (cm2) that differs from the plan's objects (0 when under 25 cm2)."""
        import e02_bricks as BR
        try:
            frames = self.still_frames(tries=3)
            pos = []
            for n in names:
                q = self.settle(min_s=0.3) if n == robot else self.st["positions"].get(n)
                if q is not None:
                    pos.append(tuple(q[:2]))
            now = BR.find_bricks(self.cam, frames, floor, self.board_rect(), robots=pos)
            g = found["grid"]
            X, Y = g.centres()
            hidden = np.zeros(g.shape, bool)              # round a robot the camera sees nothing: not compared
            for q in pos + [tuple(self.st["positions"][n][:2]) for n in names if self.st["positions"].get(n)]:
                hidden |= np.hypot(X - q[0], Y - q[1]) < 1.5 * BR.ROBOT_CM + 1.0
            # cells more than 2 cm from the other picture's objects: something new, or something gone
            new = now["occ"] & (BR.brick_distance(found["occ"], g) > 2.0) & ~hidden
            gone = found["occ"] & (BR.brick_distance(now["occ"], g) > 2.0) & ~hidden
            diff = float(new.sum() + gone.sum()) * g.cell ** 2
            return round(diff, 0) if diff >= 25.0 else 0
        except Exception as e:           # a check must never stop the demo
            self.say("  (could not check the objects: %s)" % e, quiet=True)
            return 0

    def still_frames(self, tries=6):
        """Pictures of the board once nothing moves in it any more (Nick's hands gone)."""
        import e02_bricks as BR
        region = None
        a = b = []
        for k in range(tries):
            time.sleep(1.0)
            a = self.feed.wait_frames(6, timeout=5)
            time.sleep(0.8)
            b = self.feed.wait_frames(6, timeout=5)
            if len(a) < 3 or len(b) < 3:
                continue
            if region is None:
                region = BR.board_mask(self.cam, a[0].shape, self.board_rect()) > 0
            share = BR.moving_share(a, b, region=region)
            if share < 0.004:
                return a + b
            self.say("  Something is still moving in the picture (a hand or a shadow?): please step back from the "
                     "board. Looking again ...")
        return a + b

    def save_bricks_picture(self):
        try:
            import cv2
            import e02_view
            img = self.feed.last_img
            if img is None or not getattr(self, "bricks_view", None):
                return None
            v = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img.ndim == 2 else img.copy()
            e02_view.draw_bricks(v, self.cam, self.bricks_view)
            path = os.path.join(self.dir, "check_bricks.png")
            cv2.imwrite(path, v)
            return path
        except Exception as e:                # a picture must never stop the demo
            self.say("  (brick picture not saved: %s)" % e, quiet=True)
            return None

    def bricks_to(self, robot, space, target, final_heading=None):
        """Camera-steered (not measured) to target along a path round the bricks and the other robots. Returns
        (ok, words)."""
        import e02_bricks as BR
        p = self.settle(min_s=0.3)
        if p is None:
            p = self.refind(robot, "the camera can't see it")
            if p is None:
                return False, "the camera can't see it"
        plan = BR.plan_path(space, p, target, escape_cm=25.0)
        if not plan["ok"]:
            return False, plan["why"]
        pts = plan["points"][1:]
        for k, w in enumerate(pts):
            last = k == len(pts) - 1
            ok, why = self.drive_to(robot, w, final_heading=final_heading if last else None,
                                    arrive_cm=2.0 if last else 3.0, max_leg=40.0, avoid=False)
            if not ok:
                return False, why
        return True, ""

    @staticmethod
    def crossing_stop(words, p1):
        """Plain words of what ended a crossing leg early, or '' (it ran)."""
        low = (words or "").lower()
        if low.startswith("done") or low.startswith("dup"):
            return ""
        if "obstacle" in low:
            return "its front stop saw something just ahead and stopped it (%s)" % words
        if "edge" in low:
            return "its edge guard stopped it at an open edge (%s)" % words
        if low.startswith("watchdog") or "stop command" in low:   # the watchdog's stop can beat its own note
            return "the camera watchdog stopped it (%s)" % words
        if low.startswith("camera and wheels"):
            return "" if p1 is not None else "the camera lost it (%s)" % words
        return "a leg did not run as asked (%s)" % words

    def brick_crossing(self, robot, ch, space, n, method="calibrated"):
        """One crossing (rung n of the ladder) from where the robot stands now (at the start, facing along the path)
        to the goal. method "factory" / "calibrated": the legs are worked out once, from the camera's start position
        and heading, and driven on the robot's own wheels and gyro (with whatever corrections it was given: none for
        factory); the camera only watches. "camera": after every leg the camera picks the next one from where it
        sees the robot and tells the robot where it is (a fix). c["safety"]: the front stop, edge guard or camera
        watchdog ended it (counted as failed); c["end_heading"]: which way it faced at the end, from the camera."""
        import e02_bricks as BR
        how = BR.METHOD_WORDS[method]
        c = dict(n=n, method=method, how=how, failed="", safety=False, legs_done=0, legs=[])
        turned = 0.0                  # turns sent since the last go leg the camera saw move (for end_heading)
        last_go = None
        p = self.settle(min_s=0.5)
        h, measured = self.heads.get(robot, [None, False])
        if p is None or h is None:
            c["failed"] = "the camera did not see where it starts, or which way it faces"
            return c
        # heading fix 4: the runner's heading only if measured; else its own gyro's (fix_now decides)
        self.fix_now(robot, h if measured else None)
        goal = tuple(ch["goal"])
        pts = [tuple(p)] + [tuple(q) for q in ch["path"]["points"][1:]]
        c.update(start_cam=[round(p[0], 2), round(p[1], 2)], start_heading=round(h, 1), heading_measured=bool(measured))
        self.say("  --- Crossing %d: %s ---" % (n, how))
        self.say("  %s" % ({"factory": "Factory settings: no corrections at all (gyro scale 1, drift 0, wheel scale 1). "
                                       "The camera only watches; the front stop is there if it drifts into an object.",
                            "calibrated": "The camera only watches and measures now: the robot drives the planned legs "
                                          "on its own wheels and gyro, with the corrections it learned."}.get(
            method, "After every leg the camera says where the robot really is, and the next leg is worked out from "
                    "there.")))
        self.mark("e02 bricks crossing %d start %s" % (n, robot))
        t0 = t_last = time.time()
        track = []
        self.bricks_view.setdefault("tracks", {})[n] = track

        def keep_track():
            nonlocal t_last
            now = time.time()
            track.extend((round(q[1], 2), round(q[2], 2)) for q in self.history.since(t_last, now))
            t_last = now

        def say_leg(k, total, cmd):
            if cmd[0] == "turn":
                w = "turn %.0f deg %s" % (abs(cmd[1]), "left" if cmd[1] > 0 else "right")
            else:
                w = "drive %.0f cm %s" % (abs(cmd[1]), "forward" if cmd[1] >= 0 else "back")
            self.say("    leg %d%s: %s" % (k, (" of %d" % total) if total else "", w))
        def seen(cmd, p0, p1):
            nonlocal turned, last_go
            if cmd[0] == "turn":
                turned += float(cmd[1])
            elif cmd[0] == "go" and p0 is not None and p1 is not None and math.hypot(p1[0] - p0[0], p1[1] - p0[1]) >= 3.0:
                last_go = ST.bearing(p0, p1) if cmd[1] > 0 else ST.bearing(p1, p0)
                turned = 0.0
        if method != "camera":
            legs = BR.legs_from(p, h, pts)
            c["planned_legs"] = [lg["cmd"] for lg in legs]
            for k, lg in enumerate(legs, 1):
                say_leg(k, len(legs), lg["cmd"])
                if method == "factory" and robot not in self.moves:
                    self.begin_moves(robot)                  # (re)starting moves sets its learned corrections:
                    self.set_corrections(robot, None)        # factory values again for this crossing
                p0, p1, words = self.leg(robot, lg["cmd"])
                keep_track()
                seen(lg["cmd"], p0, p1)
                c["legs"].append(dict(cmd=lg["cmd"], result=words, cam_end=p1))
                why = self.crossing_stop(words, p1)
                if why:
                    c["failed"] = why
                    break
                c["legs_done"] += 1
        else:
            steer = ST.Steer(pts[1:], heading=h, measured=bool(measured), arrive_cm=2.0, max_leg=30.0, max_legs=40)
            k = 0
            while True:
                q = self.settle(min_s=0.3, timeout=2.0)
                if q is None:
                    c["failed"] = "the camera lost it"
                    break
                cmd = steer.next(q)
                if cmd[0] == "done" or (cmd[0] == "turn" and abs(cmd[1]) < ST.MIN_TURN_DEG):
                    break
                if cmd[0] == "gave_up":
                    c["failed"] = "it did not reach the goal in %d legs" % steer.max_legs
                    break
                k += 1
                say_leg(k, 0, cmd)
                p0, p1, words = self.leg(robot, cmd)
                keep_track()
                seen(cmd, p0, p1)
                c["legs"].append(dict(cmd=cmd, result=words, cam_end=p1))
                why = self.crossing_stop(words, p1)
                if why:
                    c["failed"] = why
                    break
                c["legs_done"] += 1
                if words.lower().startswith("camera and wheels"):
                    steer.heading, steer.measured = None, False       # found again by a turn: heading unknown
                    steer.legs += 1
                    continue
                if p0 is None or p1 is None:
                    c["failed"] = "the camera lost it"
                    break
                steer.after(cmd, p0, p1)
                self.heads[robot] = [steer.heading, steer.measured]
                self.fix_now(robot, steer.heading if steer.measured else None)    # the camera fix
        time.sleep(0.3)
        endp = self.settle(t_from=time.time() - 0.8, min_s=0.5)
        keep_track()
        c["safety"] = bool(c["failed"]) and any(w in c["failed"] for w in ("front stop", "edge guard", "watchdog"))
        if c["safety"]:
            c["failed"] = "a safety stop: " + c["failed"]
        if last_go is not None:
            c["end_heading"] = round(ST.wrap180(last_go + turned), 1)
        elif method == "camera" and self.heads.get(robot, [None, False])[1]:
            c["end_heading"] = self.heads[robot][0]
        c["seconds"] = round(time.time() - t0, 1)
        self.mark("e02 bricks crossing %d end %s" % (n, "failed" if c["failed"] else "done"))
        if endp is not None:
            c["cam_end"] = [round(endp[0], 2), round(endp[1], 2)]
            c["end_err_cm"] = round(math.hypot(endp[0] - goal[0], endp[1] - goal[1]), 2)
        fin = self.listener.leg_final(robot, "-", self.main_lid.get(robot)) or {}
        bx, by = num(fin.get("x_cm")), num(fin.get("y_cm"))
        if bx is not None and by is not None:
            c["robot_end"] = [round(bx, 2), round(by, 2)]
            c["robot_thought_cm"] = round(math.hypot(bx - goal[0], by - goal[1]), 2)
        c["closest"] = BR.closest_approach(space, track)
        c["track"] = track
        return c

    def _bricks_off(self, flag):
        if self.a.dry_run and os.path.exists(flag):
            os.remove(flag)                          # the simulator takes the bricks off
            time.sleep(1.0)

    def _bricks_end(self, robot, out, flag, space):
        """Bricks off (Nick), a fresh floor memory, the robot parked; bricks.json and the plain summary."""
        import e02_bricks as BR
        p = self.settle(min_s=0.3)
        if p is not None:
            self.st["positions"][robot] = [p[0], p[1], self.heads.get(robot, [None])[0]]
        print("\n" + "-" * 78)
        self.say("The crossings are over. Take the objects off the board now (don't move the robots), step back so "
                 "your hands are out of the picture, and press Enter: %s then parks itself." % robot)
        print("-" * 78)
        if self.a.auto:
            ans = ""
            self._bricks_off(flag)
        else:
            ans = self.ask_timeout("Press Enter when the objects are off (or type k and press Enter to leave them: it "
                                   "then goes round them to its spot): ", self.a.help_wait_min * 60.0)
        kept = ans is None or ans.lower().startswith("k")
        self.mark("e02 bricks %s" % ("kept" if kept else "off"))
        self.bricks_view = None
        if not kept:
            self.bricks_guard = None                 # nothing left on the board to guard
            time.sleep(1.0)
            self.fresh_floor()
        if self.activate(robot):
            if kept:
                i = self.park_index(robot)
                ok, why = self.bricks_to(robot, space, self.board.stage(i))
                if not ok:
                    self.say("  It could not find a way round the objects to its spot (%s)." % why)
            self.set_corrections(robot, self.move_corrections(robot))
            ok, why = self.park(robot)
        else:
            ok, why = False, "the camera can't see it"
        if not ok:
            self.help_from_nick(robot, why)
        self.bricks_guard = None
        out["bricks_taken_off"] = not kept
        summary = []
        for c in out["crossings"]:
            summary.append(dict(n=c["n"], method=c.get("method"), how=c["how"], end_err_cm=c.get("end_err_cm"),
                                robot_thought_cm=c.get("robot_thought_cm"), failed=c.get("failed") or "",
                                safety_stop=bool(c.get("safety")),
                                closest_centre_cm=(c.get("closest") or {}).get("centre_cm"),
                                closest_side_cm=(c.get("closest") or {}).get("body_cm"), legs=c.get("legs_done"),
                                seconds=c.get("seconds")))
        out["summary"] = summary
        BR.save(os.path.join(self.dir, "bricks.json"), out)
        self.st["bricks_demo"] = dict(t=out["t"], robot=robot, n_bricks=len(out["bricks"]), crossings=summary,
                                      tight_gap=out.get("tight_gap", False),
                                      gap_cm=(out.get("gap") or {}).get("width_cm"),
                                      narrowest_gap_cm=((out.get("precision") or {}).get("camera") or {}).get("gap_cm"))
        self.st.save()
        print("")
        self.say("=== Brick demo: what happened ===")
        for line in BR.brick_words(out["bricks"]):
            self.say(line)
        if out.get("precision"):
            for line in BR.precision_words(robot, out["precision"]):
                self.say(line)
        if out.get("gap"):
            self.say("  It was asked to cross a %.1f cm gap (as the camera sees it)." % out["gap"]["width_cm"])
        for c in out["crossings"]:
            for line in BR.crossing_words(c):
                self.say(line)
        if summary:
            self.say("  The three crossings side by side (same path; distance from the goal by the camera; closest "
                     "approach = the gap between its side and the nearest object):")
            for line in BR.ladder_words(out["crossings"]):
                self.say(line)
        self.say("  Saved: %s" % os.path.join(self.dir, "bricks.json"))
        return out

    def switches_back(self, robot):
        """--switches-back: does this robot wear the one attachment with the switches at the back (all robots, or
        the names given)?"""
        v = getattr(getattr(self, "a", None), "switches_back", None)
        if not v:
            return False
        return v == "all" or robot in [x.strip() for x in v.split(",")]

    def safety_list(self, robot):
        """The settings safety_settings() sends and reads back for this robot (+ BACK_SETTINGS: --switches-back)."""
        return SAFETY_SETTINGS + (BACK_SETTINGS if self.switches_back(robot) else ())

    def safety_settings(self, names):
        """The edge guard on and the bare front (no nose bracket fitted), sent and read back with `get`; with
        --switches-back also the back switches and the reverse find (BACK_SETTINGS; firmware 2026-10-25b only). A
        robot that does not confirm them gets them again; still not: the session refuses to start."""
        old = [n for n in names if self.switches_back(n) and self.listener.firmware.get(n) not in BACK_FW]
        if old:
            raise SystemExit("--switches-back: %s run%s older firmware (%s); the reverse find needs %s. Upload "
                             "firmware/home_lab to it (Arduino IDE: open home_lab.ino, pick the robot's port, press "
                             "Upload), or leave it out of --switches-back." % (
                                 ", ".join(old), "s" if len(old) == 1 else "",
                                 "/".join(sorted({str(self.listener.firmware.get(n)) for n in old})), " or ".join(BACK_FW)))
        for attempt in range(3):
            todo = []
            for n in names:
                t0 = time.time()
                want = self.safety_list(n)
                for kv in want:
                    self.send(n, "set %s %g" % kv)
                    time.sleep(0.05)
                for k, _ in want:
                    self.send(n, "get %s" % k)
                    time.sleep(0.05)
                time.sleep(1.0)
                got = self.listener.got_params(n, t0)
                bad = [k for k, v in want if got.get(k) is None or abs(got[k] - v) > 1e-3]
                if bad:
                    todo.append((n, bad))
            if not todo:
                self.say("Edge guard on and no nose bracket set on %s (read back)." % ", ".join(names))
                self.st["setup"]["safety_settings"] = dict(SAFETY_SETTINGS)
                back = [n for n in names if self.switches_back(n)]
                if back:
                    self.say("Back switches on and the find backing into the boards on %s (read back)." % ", ".join(back))
                    self.st["setup"]["switches_back"] = back
                return
            names = [n for n, _ in todo]
        raise SystemExit("These robots did not confirm the edge guard settings: %s. Check they are switched on and "
                         "on the WiFi, then start this again." % "; ".join("%s (%s)" % (n, ", ".join(b)) for n, b in todo))

    # --- the find check (setup) --------------------------------------------------------------------
    def board_params(self):
        """The board settings every find and route needs (firmware sides are -1 / +1), the route start spot
        (as near the y-board as still lets it turn: the most room for the 100 cm straight), the L's turn
        towards the board's middle, and the same stand-still after every leg for every condition (camera
        fixes are only taken while still: KC needs it, the others get it too so all four drive the same way)."""
        b = self.board
        xs, ys = b.firmware_sides()
        sx, sy, _ = b.nominal_start()
        return ["set e02_xboard_side %d" % xs, "set e02_yboard_side %d" % ys,
                "set e02_xboard_cm %.2f" % b.xboard_cm, "set e02_yboard_cm %.2f" % b.yboard_cm,
                "set e02_pause_ms %d" % PAUSE_MS, "set e02_start_x %.2f" % sx, "set e02_start_y %.2f" % sy,
                "set e02_l_turn %d" % (1 if b.turn_sign > 0 else -1), "set e02_leg_pause_ms %d" % (self.leg_pause_ms() if hasattr(self, "leg_pause_ms") else LEG_PAUSE_MIN_MS),
                "set e02_transit_max_cm %d" % TRANSIT_MAX_CM,
                "set e02_find_stage_cm %g" % getattr(b, "find_stage_cm", PL.FIND_STAGE_CM)] + SAFETY_CMDS

    def fw_border(self, names, board=None):
        """The robot's own virtual border (firmware 2026-10-25a): its open edges and border_cm. Sent whenever the
        board is (re)known; robots on older firmware are skipped (they don't have it). During explore the guard's
        board (the calibration file's) stands in until explore's own."""
        b = board or self.board or (self.guard[1] if getattr(self, "guard", None) else None)
        if b is None or self.listener is None:
            return
        ox = next((v for a_, v in b.open_edges() if a_ == "x"), -999.0)
        oy = next((v for a_, v in b.open_edges() if a_ == "y"), -999.0)
        xs, ys = b.firmware_sides()
        cmds = ["set e02_xboard_side %d" % xs, "set e02_yboard_side %d" % ys, "set e02_open_x_cm %.2f" % ox,
                "set e02_open_y_cm %.2f" % oy, "set border_cm %g" % self.fw_border_cm(), "set border_seek 0"]
        for n in names:
            if self.listener.firmware.get(n) not in BORDER_FW:
                continue
            for c in cmds:
                self.send(n, c)
                time.sleep(0.03)

    def fw_border_cm(self):
        """The robots' own border: FW_BORDER_CM away, HOME_FW_BORDER_CM with --home."""
        return HOME_FW_BORDER_CM if getattr(getattr(self, "a", None), "home", False) else FW_BORDER_CM

    def fw_seek(self, robot, on):
        """border_seek for the robot's next leg (a crawl looking for the edge on purpose)."""
        if self.listener is not None and self.listener.firmware.get(robot) in BORDER_FW:
            self.send(robot, "set border_seek %d" % (1 if on else 0))

    def find_check(self):
        """One `run find` per robot before the session: does it work, and where does the camera see each
        robot touching each board (a second measure of the board lines)."""
        res = {}
        for n in self.st["robots"]:
            if n in self.unavailable:
                continue
            self.say("Find check: %s finds itself against both boards ..." % n)
            if not self.activate(n):
                self.say("  The camera can't see %s at its spot." % n)
                continue
            self.set_corrections(n, self.move_corrections(n))
            for c in self.board_params():
                self.send(n, c)
                time.sleep(0.05)
            out = self.do_find(n, "setup")
            res[n] = out
            self.say("  %s: %s" % (n, out["words"]))
            ok, why = self.park(n)
            if not ok and not self.help_from_nick(n, why):
                continue
        self.active = None
        lines = defaultdict(list)
        for n, o in res.items():
            for which, v in (o.get("board_lines") or {}).items():
                lines[which].append(v)
        img_lines = self.st["setup"]["board_lines_image"]
        summary = {}
        for which in ("x", "y"):
            v = lines.get(which) or []
            if not v:
                continue
            med = float(np.median(v))
            spread = float(np.ptp(v)) if len(v) > 1 else 0.0
            diff = med - img_lines[which]["value"]
            summary[which] = dict(touch=round(med, 2), spread=round(spread, 2), picture=img_lines[which]["value"],
                                  difference=round(diff, 2), n=len(v))
            self.say("  %s-board: where the robots touched it %.2f cm (%d robots, within %.1f cm); in the picture "
                     "%.2f cm" % (which, med, len(v), spread, img_lines[which]["value"]))
            if abs(diff) > 1.5:
                self.say("  WARNING: the two measures of the %s-board differ by %.1f cm (in the data; the picture's "
                         "line is kept)." % (which, diff))
            if img_lines[which].get("from_edge") and len(v) >= 2 and spread <= 1.0:
                self.say("  -> using where the robots touched it (%.2f cm): the picture gave no clear line." % med)
                if which == "x":
                    self.board.xboard_cm = med
                else:
                    self.board.yboard_cm = med
        self.st["board"] = self.board.to_dict()
        self.tracker_area()
        self.st["setup"]["find_check"] = dict(robots=res, lines=summary)
        self.st.save()
        self.correct_map(summary)

    def correct_map(self, lines):
        """Mapping's board points assume where the front stop halts a robot (MAP.TOF_AHEAD_CM + SAFE_FRONT_CM ahead
        of the axle); the find check's bumps measure the boards directly. Their difference is the real stopping
        distance: the map's board points are moved by it and fitted again (2026-10-22)."""
        m = self.st["setup"].get("map")
        if not m or not lines:
            return
        offs = [m["felt_boards"][w] - lines[w]["touch"] for w in ("x", "y")
                if w in (m.get("felt_boards") or {}) and w in lines]
        if not offs:
            return
        off = float(np.mean(offs))
        # felt beyond the boards means the robot stops nearer the board than assumed: move the points back
        for r in m["rays"]:
            if r.get("kind") == "board" and r.get("point") and r.get("robot_end"):
                th = math.radians(r["robot_end"][2])
                r["point"] = [r["point"][0] - off * math.cos(th), r["point"][1] - off * math.sin(th)]
        pts = [(r["point"][0], r["point"][1], r["kind"]) for r in m["rays"] if r.get("point")]
        others = [tuple(q[:2]) for n, q in self.st["positions"].items() if n != m["robot"] and q]
        m["sides"] = MAP.fit_sides(pts, self.board, others=others)
        m["board_offset_cm"] = round(off, 2)
        m["felt_boards_corrected"] = {}
        for sd in m["sides"].values():
            if sd["kind"] == "board":
                m["felt_boards_corrected"]["x" if sd["axis"] == "y" else "y"] = sd["value"]
        self.say("  Mapping vs the bumps: the front stop halts a robot %.1f cm %s a board than assumed; the map's "
                 "board points are moved by that (%s)." % (abs(off), "nearer" if off > 0 else "farther from",
                                                          ", ".join("%s-board %.1f cm" % kv for kv in
                                                                    sorted(m["felt_boards_corrected"].items()))))
        self.st.save()
        MAP.save(os.path.join(self.dir, "map.json"), m["robot"], m["rays"], m["sides"], self.board)

    def do_find(self, robot, order):
        """A find: `run find` against the two boards (do_board_find), or the edge find (any two boundaries, no
        camera) with --edge-find or once a board has been found missing; a board find that met an open edge where
        the board should be is followed by the edge find (Nick, 2026-10-23: the remove-a-board test)."""
        if getattr(self.a, "edge_find", False) or self.board.missing:
            return self.edge_find(robot, order)
        out = self.do_board_find(robot, order)
        tried = None
        if not out["ok"] and find_hold_kind(out.get("result")):
            # backing in found no board: the runner's own recovery first (Nick, 2026-10-25 11:05: "contacting the
            # user should be a last resort"); only what it cannot settle reaches the hold below
            out, settled, tried = self.find_recover(robot, order, out)
            if settled:
                return out
        holds = 0
        while not out["ok"] and find_hold_kind(out.get("result")) and holds < FIND_HOLD_MAX:
            # backing in found no board, and the recovery ladder did not settle it: stop everyone and ask Nick
            holds += 1
            ans = self.find_hold(robot, out, tried if holds == 1 else None)
            if ans == "a":
                self.say("    %s: Nick: the board is not there: finding itself against any two boundaries instead "
                         "(the edge find)" % robot)
                ef = self.edge_find(robot, order)
                ef["board_find"] = out
                return ef
            if ans != "c":
                out["words"] = "find stopped: %s (Nick: %s)" % (find_hold_kind(out.get("result")),
                                                                 "leave it out" if ans == "o" else "stop")
                return out
            self.say("    %s: trying the find again after a fresh camera fix" % robot)
            if not self.activate(robot):
                out["words"] = "find stopped: the camera can't see it after the hold"
                return out
            out = self.do_board_find(robot, order)      # its first step is a fresh camera fix
        if out.get("ok") and out.get("board_lines"):
            self.learn_board_lines(robot, out["board_lines"])
        why = "%s %s %s" % (out.get("result"), out.get("runend"), self.listener.latest("FIND", robot, out["t0"]) or "")
        if not out["ok"] and "plywood edge" in why:
            self.say("    %s: an open edge where the map has a board: finding itself against any two boundaries "
                     "instead" % robot)
            ef = self.edge_find(robot, order)
            ef["board_find"] = out
            return ef
        return out

    # --- recovery ladder for a backing-in find that found no board (Nick, 2026-10-25 11:05) -------------------
    FIND_LOOK_TOL_CM = 15.0       # chosen: the look sees the board if the front sensor reads no more than this past
    # where the map puts it (the firmware's look allows e02_find_see_cm 20 from the staging point; nearer here).
    # DOE: doe-backlog.md "Look allowance" (the same readings with the board in and lifted out set both)
    FIND_LINE_TOL_CM = 3.0        # chosen: the camera's board line within this of the setup's = the board is there
    FIND_LOOK_ROOM_CM = 15.0      # the look's turn on the spot only this far (axle) from the board's line: the body
    # with the back attachment sweeps ~7.7 cm when it turns (geometry, no DOE)
    FIND_RESTAGE_CM = 35.0        # chosen: the second try starts this far out from the board, at another place
    FIND_RESTAGE_ALONG_CM = 25.0  # ... at least this far along it from where the first try started

    def find_recover(self, robot, order, out):
        """The runner's own recovery steps after "no board seen" / "no board behind" (each logged in
        find_recoveries and said): (1) a fresh camera fix (its position, and its heading only if measured);
        (2) look: turn to face where the board should be and read the front sensor, and re-measure that board's line
        in the camera picture; (3) if either says the board is there: the find again from another staging point
        (FIND_RESTAGE_CM out, elsewhere along the board); (4) if both say it is gone: the map marks it missing (an open
        edge from now on, as the edge find does: the virtual border keeps the robots off it, and routes are
        rechecked) and the robot finds itself against the other boundaries (the edge find). Returns (out, settled,
        tried): settled True if no hold is needed; tried is the list of plain-word steps for the hold's message."""
        res = out.get("result") or ""
        kind = find_hold_kind(res)
        m = re.search(r"\b([xy]) board", res)
        which = m.group(1) if m else None
        rec = dict(t=round(time.time(), 1), robot=robot, order=order, kind=kind, board=which, steps=[])
        self.st.setdefault("find_recoveries", []).append(rec)
        tried = []

        def step(name, words, **kw):
            rec["steps"].append(dict(step=name, words=words, **kw))
            tried.append(words)
            self.say("    %s: find recovery (%s): %s" % (robot, name, words))
            self.mark("e02 find recovery %s %s" % (robot, name))
        self.say("  %s: the find found no board (%s); trying to sort it out before calling Nick" % (robot, kind))
        # (1) a fresh camera fix
        p = self.fix_now(robot) if self.activate(robot) else None
        step("fix", "a fresh camera fix: %s" % ("it is at (%.0f, %.0f) cm" % tuple(p[:2]) if p is not None
                                                 else "the camera could not see it"), ok=p is not None)
        if p is None or which is None:
            return out, False, tried
        # (2) look: the front sensor facing the board, and the camera picture of its line
        front, expect = self.find_look(robot, which, p)
        seen_front = None if front is None and expect is None else (
            front is not None and expect is not None and front <= expect + self.FIND_LOOK_TOL_CM)
        step("look", "facing the %s-board its front sensor read %s (the board should be about %s)" % (
            which, "nothing" if front is None else "%.0f cm" % front, "?" if expect is None else "%.0f cm" % expect),
            front_cm=front, expected_cm=None if expect is None else round(expect, 1), sees=seen_front)
        line = self.camera_board_line(which)
        ref = ((self.st.get("setup") or {}).get("board_lines_image") or {}).get(which) or {}
        ref_v = ref.get("value") if not ref.get("from_edge") else None
        if ref_v is None:
            ref_v = self.board.xboard_cm if which == "x" else self.board.yboard_cm
        seen_cam = None if line is False else (line is not None and abs(line - ref_v) <= self.FIND_LINE_TOL_CM)
        step("picture", "the camera picture %s" % (
            "could not be checked" if line is False else "shows no line where the %s-board stands" % which
            if line is None else "shows the %s-board's line at %.1f cm (%s %.1f at setup)" % (
                which, line, "as" if seen_cam else "not as", ref_v)), line_cm=None if line in (None, False) else
            round(line, 2), sees=seen_cam)
        # (4) both say it is gone: the map changes, and the robot finds itself against the other boundaries
        if seen_front is False and seen_cam is False:
            self.board_gone(robot, which, "the front sensor and the camera picture both say so")
            step("missing", "the front sensor and the camera both say the %s-board is gone: the map now has an open "
                            "edge there, and %s finds itself against the other boundaries" % (which, robot))
            ef = self.edge_find(robot, order)
            ef["board_find"] = out
            ef["recovery"] = rec
            return ef, True, tried
        # (3) the board looks there (or one check could not tell): the find again from another staging point
        spot = self.restage_spot(which, p)
        if spot is not None:
            ok_, why_ = self.drive_to(robot, spot, max_leg=PARK_LEG_CM, arrive_cm=4.0)
            self.end_moves(robot)
        else:
            ok_, why_ = False, "no room for another staging point"
        if ok_:
            out2 = self.do_board_find(robot, order)
            step("retry", "the find again from (%.0f, %.0f) cm: %s" % (spot[0], spot[1], "it worked" if out2["ok"] else
                                                                        out2.get("result") or out2["words"]),
                 ok=bool(out2["ok"]))
            out2["recovery"] = rec
            if out2["ok"] or not find_hold_kind(out2.get("result")):
                return out2, out2["ok"], tried
            out = out2
        else:
            step("retry", "could not move to another staging point (%s)" % why_, ok=False)
        return out, False, tried

    def find_look(self, robot, which, p):
        """Recovery step 2: turn to face the board (the x-board is a line y = xboard_cm, the y-board x = yboard_cm)
        and read the front sensor. Returns (reading cm or None, where the map puts the board from the sensor)."""
        b = self.board
        line = b.xboard_cm if which == "x" else b.yboard_cm
        here = p[1] if which == "x" else p[0]
        face = (90.0 if line > here else -90.0) if which == "x" else (0.0 if line > here else 180.0)
        own = self.own_pose(robot)
        h = own[2] if own is not None else (self.heads.get(robot) or [None])[0]
        if h is None or not self.begin_moves(robot, "looking for the %s-board" % which):
            return None, None
        if abs(line - here) < self.FIND_LOOK_ROOM_CM:
            # right against the board ("no board behind" leaves it there, facing away): turning on its spot would
            # sweep its corners into the board, so first forward, away from it
            if math.cos(math.radians(h - face)) > -0.7:
                self.end_moves(robot)
                return None, None                 # not facing away: no safe way to make room
            self.leg(robot, ("go", self.FIND_LOOK_ROOM_CM - abs(line - here) + 2.0))
            q = self.settle(min_s=0.3, timeout=2.0)
            if q is not None:
                here = q[1] if which == "x" else q[0]
        t = ST.wrap180(face - h)
        if abs(t) >= ST.MIN_TURN_DEG:
            self.leg(robot, ("turn", t))
        f = self.front_now(robot)
        off, _tol = self.front_limits(robot)
        expect = abs(line - here) - PL.ROBOT_HALF_CM + off
        self.end_moves(robot)
        return f, expect

    def camera_board_line(self, which):
        """Recovery step 2: the board's floor line in the camera's newest picture (cm), None if no line is seen
        there, False if there is no picture to check."""
        img = getattr(self.feed, "last_img", None) if self.feed is not None else None
        if img is None or self.cam is None:
            return False
        try:
            m = CAM.measure_board_line(img, self.cam, self.board, which)
        except Exception:
            return False
        return None if m is None else float(m["value"])

    def restage_spot(self, which, p):
        """Recovery step 3: another staging point for the find: FIND_RESTAGE_CM out from the board, at least
        FIND_RESTAGE_ALONG_CM along it from where the robot is, on the board and away from the open edges."""
        b = self.board
        line = b.xboard_cm if which == "x" else b.yboard_cm
        here_n = p[1] if which == "x" else p[0]
        out_n = line - self.FIND_RESTAGE_CM if line > here_n else line + self.FIND_RESTAGE_CM
        lo, hi = (b.x0, b.x1) if which == "x" else (b.y0, b.y1)
        lo, hi = min(lo, hi), max(lo, hi)
        here_a = p[0] if which == "x" else p[1]
        mid = (lo + hi) / 2.0
        cands = [mid, mid - 30.0, mid + 30.0, lo + 30.0, hi - 30.0]
        for a in sorted(cands, key=lambda v: -abs(v - here_a)):
            if abs(a - here_a) < self.FIND_RESTAGE_ALONG_CM:
                continue
            x, y = (a, out_n) if which == "x" else (out_n, a)
            if b.on_board(x, y, margin=-15.0) and b.edge_room(x, y)[0] >= 20.0:
                return (x, y)
        return None

    def board_gone(self, robot, which, why):
        """The map marks a board missing (an open edge from now on), as the edge find does when it feels one."""
        if which in self.board.missing:
            return
        self.board.missing.add(which)
        v = self.board.xboard_cm if which == "x" else self.board.yboard_cm
        self.st.setdefault("board_changes", []).append([round(time.time(), 1), robot, "%s board gone (%s)" % (which, why),
                                                        v])
        self.recheck_routes("the %s board is gone" % which)
        self.say("    the %s-board is gone (%s): the map now has an open edge there; the robots keep the virtual "
                 "border's distance from it" % (which, why))
        self.mark("e02 board gone %s" % which)
        self.st["board"] = self.board.to_dict()

    def find_hold(self, robot, out, tried=None):
        """A backing-in find that found no board (firmware 2026-10-25b: "no board seen", the look before turning
        round; "no board behind", the backing cap): hold everyone and ask Nick (hold_for_nick, kind 'find') with
        what happened, which board, where the robot is and what to check. Returns his answer: c (try again after a
        fresh camera fix), a (the board is gone: the edge find), o (leave it out), s (stop)."""
        res = out.get("result") or ""
        why, detail = find_hold_words(robot, res, self.find_hold_where(robot), self.board)
        if tried:
            detail = "What the runner tried first: " + "; ".join(tried) + ". " + detail
        self.st.setdefault("find_holds", []).append([round(time.time(), 1), robot, find_hold_kind(res), res])
        return self.hold_for_nick(robot, "find", why, detail=detail)

    def find_hold_where(self, robot):
        """(x, y, heading or None, where from) of the robot for a find hold: the camera's, else its own pose."""
        q = None
        try:
            w = self.pos(robot, fresh_s=3.0)              # tracker.where(): (t, x, y, ...)
            if w is not None:
                q = (w[1], w[2])
            elif self.tracker is not None and self.tracker.active == robot:
                q = self.settle(min_s=0.3, timeout=2.0)   # (x, y)
        except Exception:
            q = None
        h = (self.heads.get(robot) or [None])[0]
        if q is not None:
            return (float(q[0]), float(q[1]), h, "the camera")
        own = self.own_pose(robot) if self.listener is not None else None
        if own is not None:
            return (own[0], own[1], own[2], "its own wheels and gyro")
        return None

    BOARD_LEARN_MIN_N = 3          # chosen: three touches before a board line moves (one odd touch must not)
    BOARD_LEARN_STEP_CM = 0.5      # chosen: smaller differences are left (the camera's own noise)
    BOARD_LEARN_CAP_CM = 8.0       # Nick, 2026-10-23: never more than 8 cm from the picture's line
    BOARD_LEARN_LAST = 30          # the newest touches only (a board bumped mid-session is followed)

    def learn_board_lines(self, robot, lines):
        """Item 3 (Nick, 2026-10-23: option (a) revised): the map's board lines move to where the robots feel them.
        Session 1: every robot touched both boards ~4 cm before its own pose expected, the same at every find: the
        map's lines, not the robots. Each finished board find adds where the camera saw it press against each board;
        each robot's steady difference from the others is learned too (its own residual, median of its touches
        against everyone's) and taken out before the shared line is formed, so one robot's oddity does not move
        the board; it is reported per robot (not applied to its pose: it may be how the camera sees that robot's
        top, not where the robot is; to be checked against the footage). The line moves after BOARD_LEARN_MIN_N
        touches when the difference is BOARD_LEARN_STEP_CM or more, capped at BOARD_LEARN_CAP_CM from the
        picture's line; every move is logged (board_line_updates) and said."""
        st = self.st
        tl = st.setdefault("board_touches", [])
        for which, v in lines.items():
            tl.append([round(time.time(), 1), robot, which, round(float(v), 2)])
        img = (st["setup"].get("board_lines_image") or {})
        res_all = st.setdefault("robot_board_offsets", {})
        for which in ("x", "y"):
            rows = [r for r in tl if r[2] == which][-self.BOARD_LEARN_LAST:]
            if len(rows) < self.BOARD_LEARN_MIN_N:
                continue
            med = float(np.median([r[3] for r in rows]))
            # each robot's own steady difference from everyone's line
            res = {}
            for n in {r[1] for r in rows}:
                mine = [r[3] - med for r in rows if r[1] == n]
                if len(mine) >= 2:
                    res[n] = float(np.median(mine))
                    res_all.setdefault(n, {})[which] = round(res[n], 2)
            line = float(np.median([r[3] - res.get(r[1], 0.0) for r in rows]))
            pic = (img.get(which) or {}).get("value")
            if pic is not None:
                line = min(pic + self.BOARD_LEARN_CAP_CM, max(pic - self.BOARD_LEARN_CAP_CM, line))
            cur = self.board.xboard_cm if which == "x" else self.board.yboard_cm
            if abs(line - cur) < self.BOARD_LEARN_STEP_CM:
                continue
            if which == "x":
                self.board.xboard_cm = line
            else:
                self.board.yboard_cm = line
            # the finds so far put their robots against the OLD line: their true positions (the camera error map's
            # samples) move with it (2026-10-24 night replay: the first finds were 2.6-2.8 cm off after a move)
            dlt = line - cur
            ce_ = getattr(self, "camerr", None)
            for smp_ in (ce_.samples if ce_ is not None else []):
                if smp_.get("source") != "find":
                    continue
                k_ = "y" if which == "x" else "x"
                smp_[k_] = round(smp_[k_] + dlt, 2)
                smp_["d" + k_] = round(smp_["d" + k_] - dlt, 2)
            if ce_ is not None:
                st["camera_error_samples"] = ce_.to_list()
            st.setdefault("board_line_updates", []).append(
                dict(t=round(time.time(), 1), board=which, before=round(cur, 2), after=round(line, 2),
                     picture=pic, touches=len(rows), robot_offsets={k: round(v, 2) for k, v in res.items()}))
            self.say("    the %s-board's line moved %+.1f cm to where the robots feel it (%.2f cm; %d touches%s)" % (
                which, line - cur, line, len(rows), "; robots' own differences: " + ", ".join(
                    "%s %+.1f" % kv for kv in sorted(res.items())) if res else ""))
            self.mark("e02 board line %s %.2f" % (which, line))
        st["board"] = self.board.to_dict()
        self.tracker_area()
        st.save()

    def do_board_find(self, robot, order):
        """A camera fix (the firmware wants one before every find), then `run find`, and the camera's view of
        it. Returns dict(ok, words, run, board_lines, t0, t1)."""
        self.end_moves(robot)
        t0 = time.time()
        self.phase = "find"
        hfind, ponly = self.find_heading_ok(robot)
        self.fix_now(robot, heading=hfind, position_only=ponly)
        self.watchdog_fired = False
        self.watch_stop = None
        self.mark("e02 %s %s find start" % (order, robot))
        if self.switches_back(robot):
            for kv in BACK_SETTINGS:                 # again before every find: a power-cycled robot lost them
                self.send(robot, "set %s %g" % kv)
                time.sleep(0.03)
        if getattr(self, "find_live", None) is None:
            self.find_live = {}
        self.find_live[robot] = float("inf")     # driving: no still-camera evidence from it (camera_still_check)
        try:
            run = self.start_run(robot, "run find")
            if not run:
                return dict(ok=False, words="did not start `run find`", run=None, t0=t0, t1=time.time())
            end = self.watch.wait_end(run, FIND_TIMEOUT_S, poll=lambda: self.watchdog(robot))
            t1 = time.time()
            if end is None:
                self.send(robot, "stop")
                end = self.watch.wait_end(run, 5) or dict(result="no end record")
        finally:
            self.find_live[robot] = time.time()
        fr = self.listener.latest("FIND", robot, t0)
        # find ends squared up to the boards, backed off: its own heading is then as good as the camera's. Nose
        # first it ends facing the y board; backing in (FIND reverse 1, firmware 2026-10-25b) facing away from it
        rev = bool(fr and (fr[1].get("reverse") or "0").strip() == "1")
        if self.switches_back(robot) and fr and not rev:
            self.say("  %s: found itself nose first, not backing in (its back switches were not set: see its "
                     "messages); carrying on" % robot)
        ath = num(fr[1].get("after_th")) if fr else None
        if ath is None:
            ath_guess = self.board.start_heading if (rev or (fr is None and self.switches_back(robot))) else \
                self.board.start_heading + 180.0
            self.heads[robot] = [ST.wrap180(ath_guess), False]
        else:
            self.heads[robot] = [ath, True]
        result = fr[1].get("result", "?") if fr else end["result"]
        ok = end["result"] == "done" and (fr is None or result == "ok")
        out = dict(ok=ok, run=run, t0=t0, t1=t1, result=result, runend=end["result"],
                   words="found itself" if ok else "find did not finish (%s, %s)" % (result, end["result"]))
        if ok:
            self.lost_place.discard(robot)
        elif self.pos(robot, fresh_s=2.0) is None:
            # a failed find the camera did not see the end of: the robot is somewhere it drove unseen (dry run
            # 16:41: its setup find ended at an open edge while hidden; the runner kept its old place)
            self.lost_place.add(robot)
        # where the camera saw it touch each board
        lines = {}
        raw = self.listener.raw.get(run) or []
        off = FIT.clock_offset(raw)
        with self.listener.cv:
            bumps = [(t, rec) for t, n, rec in self.listener.recs["BUMP"] if n == robot and rec.get("run") == run]
        prm = self.watch.params.get(run, {})
        face = PL.ROBOT_HALF_CM + (prm.get("bumper_cm") or 0.0)      # body centre (the camera's point) to its face
        if prm.get("bumper_cm") and prm.get("bump_ahead_cm"):
            self.board.face_cm = prm["bump_ahead_cm"]
        if rev:
            # backed in: the back attachment's face touched (centre ~ axle: PL.ROBOT_HALF_CM is both); the robot
            # reaches that far behind its centre too
            face = prm.get("e02_back_face_cm") or num(fr[1].get("face_cm")) or BACK_SETTINGS[-1][1]
            self.board.face_cm = max(self.board.face_cm, face)
        for _, rec in bumps:
            ms = num(rec.get("ms_contact")) or num(rec.get("ms_detect")) or num(rec.get("ms"))
            which = (rec.get("board") or "").strip().lower()[:1]
            if ms is None or off is None or which not in ("x", "y"):
                continue
            tb = ms / 1000.0 + off
            smp = self.history.since(tb - 0.2, tb + 2.5)        # the bump, the push to square, the back-off
            sp = None
            if len(smp) >= 5:
                # where it pressed against the board: the samples closest to it
                key = (lambda q: q[2] * self.board.uy) if which == "x" else (lambda q: q[1] * self.board.ux)
                near = sorted(smp, key=key)[:max(3, len(smp) // 5)]
                sp = (float(np.median([q[1] for q in near])), float(np.median([q[2] for q in near])))
            if sp:
                if which == "x":
                    lines["x"] = sp[1] - self.board.uy * face
                else:
                    lines["y"] = sp[0] - self.board.ux * face
        out["board_lines"] = {k: round(v, 2) for k, v in lines.items()}
        if ok and fr:
            # its pose is exact against both boards now: where the camera sees it is a check of the camera there
            ax, ay = num(fr[1].get("after_x")), num(fr[1].get("after_y"))
            cam = self.settle(min_s=0.3, timeout=2.0)
            if cam is not None and ax is not None and ay is not None:
                # the camera's error here, corrected by its map from the OTHER finds (leave one out): its trust
                cx_, cy_, _ = self.camerr.corrected(cam[0], cam[1])
                self.trust.camera_at_find(robot, cx_ - ax, cy_ - ay)
                self.trust_note()
            smp = self.camerr_add(ax, ay, cam, "find", robot)
            if smp:
                out["camera_error_cm"] = round(math.hypot(smp["dx"], smp["dy"]), 2)
            if not rev and not getattr(self.a, "no_face_reading", False):
                out["face_resid_cm"] = self.face_reading(robot, fr)
        out["bumps"] = len(bumps)
        out["bump_results"] = [rec.get("result") for _, rec in bumps]
        self.mark("e02 %s %s find end %s" % (order, robot, end["result"]))
        return out

    FACE_TARGET = 8           # chosen: face readings per robot per session (= e02_front SESSION_MIN, so a session's own
                              # offset is known from them alone; about every nose-first find in a 25-min session; DOE:
                              # how soon a shifted sensor is caught at 4 / 8 / 16, in the simulator with front:bias)

    def face_reading(self, robot, fr):
        """A deliberate front-sensor reading against a known distance (2026-10-25: the front sensor's evidence was slow,
        Left had ~6 square-on readings a session). Right after a nose-first find the robot stands square to the y
        board, its pose known from touching both boards (no camera): a 2 deg turn leg reports the front sensor
        (firmware 2026-10-22b), compared with the distance to the board by that pose. Idle leg with no camera fix
        first (begin_moves would overwrite the exact pose with the camera's). Up to FACE_TARGET a session. Returns the
        residual (reading - distance, cm) or None."""
        st = self.st
        done = st.setdefault("face_readings", {}).get(robot, 0)
        if done >= self.FACE_TARGET or self.board is None or robot in self.moves:
            return None
        ax, ay, ath = num(fr[1].get("after_x")), num(fr[1].get("after_y")), num(fr[1].get("after_th"))
        if None in (ax, ay, ath):
            return None
        self.moves[robot] = dict(run="-", why="its front sensor at the board")
        try:
            f = self.front_now(robot)
        finally:
            self.moves.pop(robot, None)
        hd = ST.wrap180(ath + 2.0)
        self.heads[robot] = [hd, True]
        if f is None:
            return None
        if self.degrade.spec.get("front"):
            f = self.degrade.front(f, robot)              # --degrade: a bad front sensor
        ahead = self.ahead_cm(robot, (ax, ay), hd)
        if ahead is None or "board" not in str(ahead[1]) or ahead[0] > FRONT_CHECK_RANGE_CM or not self.square_on(ahead):
            return None
        st["face_readings"][robot] = done + 1
        off, _tol = self.front_limits(robot)
        resid = f - ahead[0]
        path = os.path.join(self.dir, "front_checks.csv")
        new = not os.path.exists(path)
        with open(path, "a", newline="") as fh:
            w = csv.writer(fh)
            if new:
                w.writerow(["t", "robot", "leg", "value", "x_cm", "y_cm", "heading_deg", "heading_measured",
                            "front_cm", "expected_cm", "what"])
            w.writerow([round(time.time(), 2), robot, "face", 2.0, round(ax, 1), round(ay, 1), round(hd, 1), 1, f,
                        round(ahead[0], 1), ahead[1]])
        fm = getattr(self, "front", None)
        if fm is not None:
            ch_ = fm.add(robot, resid)
            if ch_:
                self.say("!!! FRONT SENSOR: " + ch_ + ".")
                self.mark("e02 front sensor changed %s" % robot)
                st.setdefault("front_changes", []).append([round(time.time(), 1), robot, ch_])
                if hasattr(self.trust, "mark_changed"):
                    self.trust.mark_changed("front", robot, ch_)
        if hasattr(self.trust, "front_face"):
            self.trust.front_face(robot, resid - off)
            self.trust_note(save=False)
        self.say("    %s: front sensor %.1f cm at %s, %.1f cm by the board touches (face reading %d)" % (
            robot, f, ahead[1], ahead[0], done + 1), quiet=True)
        return round(resid, 2)

    def edge_leg(self, robot, kind, val, timeout=None):
        """One idle leg for the edge find. Returns (state, info, rec): rec the LEG record (its pose: x_cm, y_cm,
        th_deg, in the robot's own frame, kept across legs since its pose is known)."""
        est = abs(val) / DRIVE_CMS_GUESS + 1.0 if kind == "go" else abs(val) / TURN_DPS_GUESS + 1.0
        state, info = self.send_leg(robot, "%s %.2f" % (kind, val), timeout or (2.5 * est + 6.0))
        rec = self.listener.leg_final(robot, "-", self.last_lid.get(robot))
        return state, info, rec

    def edge_sides(self):
        """The map's four boundaries for the edge find: [('x' | 'y', value, 'board' | 'edge')]."""
        bd = self.board
        out = []
        for a, v in (("x", bd.x0), ("x", bd.x1), ("y", bd.y0), ("y", bd.y1)):
            is_open = any(a == oa and abs(v - ov) < 1.0 for oa, ov in bd.open_edges())
            line = v
            if not is_open:
                line = bd.yboard_cm if a == "x" else bd.xboard_cm     # a board's face, a little inside the edge
            out.append((a, line, "edge" if is_open else "board"))
        return out

    def make_way(self, robot, blocker, here, boundary):
        """Coordinated moves, first piece (Nick, 2026-10-23: "we will need for multiple robots to move in coordinated
        patterns soon"): `robot` stands still where it is (parked in the tracker's memory), `blocker` drives sideways
        out of its path to `boundary` and parks there, then the camera goes back to `robot`. Logged (make_way).
        Returns (ok, words)."""
        others = [tuple(q[:2]) for n, q in self.st["positions"].items() if n not in (robot, blocker) and q]
        others.append(tuple(here))
        b = self.st["positions"].get(blocker)
        if not b:
            return False, "where it is is not known"
        spot = EF.make_way_spot(b[0], b[1], here[0], here[1], boundary, others,
                                inside=lambda x, y: self.board.on_board(x, y, margin=-1.0)
                                and self.board.edge_room(x, y)[0] >= MAKE_WAY_EDGE_CM)
        if spot is None:
            return False, "no room beside the path"
        self.say("    %s: %s is in the way; it moves aside to (%.0f, %.0f) cm" % (robot, blocker, spot[0], spot[1]))
        self.mark("e02 make way %s for %s" % (blocker, robot))
        h = self.heads.get(robot, [None])[0]
        self.st["positions"][robot] = [here[0], here[1], h]
        self.tracker.park(robot, here[0], here[1])
        self.end_moves(robot)
        if not self.activate(blocker):
            self.activate(robot)
            return False, "the camera can't see it"
        ok, why = self.drive_to(blocker, spot, arrive_cm=3.0, max_leg=PARK_LEG_CM)
        self.end_moves(blocker)
        p = self.settle(min_s=0.3) or spot
        self.st["positions"][blocker] = [p[0], p[1], self.heads.get(blocker, [None])[0]]
        self.tracker.park(blocker, p[0], p[1])
        self.st.setdefault("make_way", []).append([round(time.time(), 1), robot, blocker, [round(v, 1) for v in b[:2]],
                                                    [round(p[0], 1), round(p[1], 1)], bool(ok), why])
        self.activate(robot)
        self.moves[robot] = dict(run="-", why="edge find")
        self.st.save()
        return ok, why

    def edge_find(self, robot, order):
        """The edge find (_edge_find), with the camera watchdog's open-edge check off for this robot throughout
        (dry run 2026-10-23: it stopped the sidestep running beside an open edge; the edge guard is the protection,
        as in explore)."""
        self.edge_touch = robot
        self.fw_seek(robot, True)
        try:
            return self._edge_find(robot, order)
        finally:
            self.fw_seek(robot, False)
            self.edge_touch = None

    def _edge_find(self, robot, order):
        """The edge find (e02_edgefind): touch boundary A twice a sidestep apart, then B once, by the robot's own
        stops (front stop at a board, edge guard at an open edge); no camera. A touch that meets the other kind of
        boundary than the map says is believed (the map is corrected: a board gone is an open edge from now on).
        Ends with `fix` (its pose in the board's frame). Returns the do_find dict."""
        t0 = time.time()
        self.end_moves(robot)
        self.phase = "find"
        self.mark("e02 %s %s edge find start" % (order, robot))
        out = dict(ok=False, run=None, t0=t0, result="failed", runend="-", board_lines={}, bumps=0, bump_results=[],
                   kind="edge")
        p = self.pos(robot, fresh_s=2.0)
        p = (p[1], p[2]) if p is not None else None        # pos() is (t, x, y)
        own = self.own_pose(robot)
        h = self.heads.get(robot, [None])[0]
        if p is None and own is not None:
            p = own[:2]
        if h is None and own is not None:
            h = own[2]
        if p is None or h is None:
            out.update(words="edge find: where it is or which way it faces is not known", t1=time.time())
            return out
        # one frame for the touches: its believed pose, set now (a pose it keeps across the legs)
        self.send(robot, "fix %.2f %.2f %.1f %.3f" % (p[0], p[1], h, time.time()))
        self.fix_t[robot] = time.time()
        time.sleep(0.15)
        self.moves[robot] = dict(run="-", why="edge find")
        sides = self.edge_sides()
        others = [tuple(q[:2]) for n, q in self.st["positions"].items() if n != robot and q]
        ab = EF.choose_boundaries(p[0], p[1], sides, others=others)
        if ab is None:
            out.update(words="edge find: no two boundaries at right angles on the map", t1=time.time())
            return out
        a, b = ab
        touches, pose = [], [p[0], p[1], h]
        found_kinds = {}

        def update(rec):
            if rec and None not in (num(rec.get("x_cm")), num(rec.get("y_cm")), num(rec.get("th_deg"))):
                pose[:] = [num(rec.get("x_cm")), num(rec.get("y_cm")), num(rec.get("th_deg"))]

        def turn_to(hdg):
            d = EF.wrap(hdg - pose[2])
            if abs(d) > 1.0:
                st_, info, rec = self.edge_leg(robot, "turn", d)
                update(rec)
                return st_ in ("done", "dup")
            return True

        moved_aside = []

        def touch(bd, on):
            # a parked robot in the way moves aside first (Nick, 2026-10-23: practice for robots moving together)
            here = self.pos(robot, fresh_s=2.0)
            hx, hy = (here[1], here[2]) if here else (pose[0], pose[1])
            others = {n: tuple(q[:2]) for n, q in self.st["positions"].items() if n != robot and q}
            for m in EF.blockers(hx, hy, bd, others):
                ok_, words_ = self.make_way(robot, m, (hx, hy), bd)
                moved_aside.append(m)
                if not ok_:
                    return "%s is in the way and could not move aside (%s)" % (m, words_)
            if not turn_to(EF.facing(bd, pose[0], pose[1])):
                return "could not turn to face the %s=%.0f side" % (bd[0], bd[1])
            gap = abs((pose[0] if bd[0] == "x" else pose[1]) - bd[1])
            st_, info, rec = self.edge_leg(robot, "go", min(100.0, gap + EF.APPROACH_EXTRA_CM))
            update(rec)
            kind = MAP.stop_kind(st_, info)
            if kind is None:
                return "drove %.0f cm towards the %s=%.0f side and met nothing" % (gap + EF.APPROACH_EXTRA_CM, bd[0], bd[1])
            touches.append(dict(x=pose[0], y=pose[1], th=pose[2], kind=kind, on=on, cue=MAP.edge_cue(info)))
            found_kinds[(bd[0], round(bd[1], 1))] = (bd[2], kind)
            # its pose as the fix first: the firmware allows e02_back_max_cm of blind backing per fix
            self.send(robot, "fix %.2f %.2f %.1f %.3f" % (pose[0], pose[1], pose[2], time.time()))
            time.sleep(0.15)
            st_, info, rec = self.edge_leg(robot, "go", -(EF.BACK_EDGE_CM if kind == "edge" else EF.BACK_CM))
            update(rec)
            return ""

        why = touch(a, "A")
        if not why:
            sgn, room = EF.sidestep_room(a, pose[0], pose[1], sides)
            step = max(EF.SIDESTEP_MIN_CM, min(EF.SIDESTEP_CM, room - 20.0))     # 20 cm clear of what is beside
            for kind, val in (("turn", sgn * 90.0), ("go", step), ("turn", -sgn * 90.0)):
                before = list(pose)
                st_, info, rec = self.edge_leg(robot, kind, val)
                update(rec)
                if st_ in ("done", "dup"):
                    continue
                moved = math.hypot(pose[0] - before[0], pose[1] - before[1])
                if kind == "go" and "plywood edge" in (info or "") and moved >= EF.SIDESTEP_MIN_CM:
                    self.send(robot, "fix %.2f %.2f %.1f %.3f" % (pose[0], pose[1], pose[2], time.time()))
                    time.sleep(0.15)
                    # stopped by an edge part way (2026-10-23 dry run: a slant took it towards the open edge it had
                    # just touched): back off it, blind, and use the sidestep it made (the two touches need only be
                    # apart, not exactly SIDESTEP_CM)
                    st2, info2, rec2 = self.edge_leg(robot, "go", -EF.BACK_CM)
                    update(rec2)
                    continue
                why = "the sidestep along the %s=%.0f side stopped (%s)" % (a[0], a[1], info)
                break
        if not why:
            why = touch(a, "A")
        if not why:
            why = touch(b, "B")
        sol = EF.solve(touches, a, b) if not why else None
        self.end_moves(robot)
        for m in moved_aside:
            # back to their spots once this robot's run is over and it has parked (2026-10-23 dry runs: left where it
            # stepped aside, Left stood in the next run's route; sent back straight away, this robot still stood
            # by its spot)
            if m not in self.return_after:
                self.return_after.append(m)
        t1 = time.time()
        rec = dict(t=round(t0, 1), robot=robot, order=order, a=list(a[:2]), b=list(b[:2]),
                   touches=touches, kinds={"%s=%g" % k: v for k, v in found_kinds.items()}, why=why)
        if sol is None:
            out.update(words="edge find failed: %s" % (why or "the two touches on one side came out too close"), t1=t1)
            rec["ok"] = False
            self.st.setdefault("edge_finds", []).append(rec)
            self.lost_place.add(robot)
            return out
        bx, by, bth = EF.to_board(sol, pose[0], pose[1], pose[2])
        self.send(robot, "fix %.2f %.2f %.1f %.3f" % (bx, by, bth, time.time()))
        self.fix_t[robot] = time.time()
        self.heads[robot] = [bth, True]
        self.lost_place.discard(robot)
        rec.update(ok=True, rot_deg=round(sol["rot"], 2), shift=[round(sol["tx"], 2), round(sol["ty"], 2)],
                   before=[round(v, 2) for v in pose], after=[round(bx, 2), round(by, 2), round(bth, 1)])
        self.st.setdefault("edge_finds", []).append(rec)
        # the map against what was felt
        for (ax, v), (was, felt) in found_kinds.items():
            if was == "board" and felt == "edge":
                which = "y" if ax == "x" else "x"            # the y board stands along an x = value line
                self.board.missing.add(which)
                self.st.setdefault("board_changes", []).append([round(t1, 1), robot, "%s board gone" % which, v])
                self.recheck_routes("the %s board is gone" % which)
                self.say("    %s felt an open edge where the map had the %s board (%s = %.0f): the map now has an open "
                         "edge there" % (robot, which, ax, v))
                self.mark("e02 board gone %s" % which)
                self.st["board"] = self.board.to_dict()
            elif was == "edge" and felt == "board":
                self.st.setdefault("board_changes", []).append([round(t1, 1), robot, "%s board felt at an open edge" % ax, v])
                self.say("    %s felt a board where the map has an open edge (%s = %.0f); noted" % (robot, ax, v))
        self.say("    %s found itself against the %s=%.0f and %s=%.0f sides (%s, %s): at (%.1f, %.1f) facing %.0f deg "
                 "(its own pose was %.1f cm, %.1f deg off)" % (
                     robot, a[0], a[1], b[0], b[1], touches[0]["kind"], touches[-1]["kind"], bx, by, bth,
                     math.hypot(bx - pose[0], by - pose[1]), EF.wrap(bth - pose[2])))
        self.mark("e02 %s %s edge find end ok" % (order, robot))
        self.st.save()
        out.update(ok=True, result="ok", words="found itself against two boundaries", t1=t1)
        return out

    # --- the virtual border and holding for Nick (Nick, 2026-10-24 evening) -------------------------------------
    def stop_everyone(self, rounds=3):
        names = set(self.st["robots"]) | set(self.link.robots() if self.link else [])
        for _ in range(rounds):
            for n in names:
                try:
                    self.link.send_name(n, "stop")
                except Exception:
                    pass
            time.sleep(0.2)

    # --- finding a robot again on its own before Nick is asked (coordinator, 2026-10-26 10:00-10:02) ---------
    AUTO_FIND_MAX = 3             # chosen: at most this many automatic finds per robot in AUTO_FIND_WINDOW_S; then Nick
    AUTO_FIND_WINDOW_S = 300.0    # (no DOE: a loop guard against a robot the tracker keeps losing)
    AUTO_FIND_SAME_CM = 6.0       # chosen = DETECT_NEAR_CM: its lights within this of the camera's spot confirm the spot

    def auto_find(self, robot, why):
        """The camera lost the robot (or doubts it): find it again WITHOUT Nick, by what it knows (2026-10-26 10:00,
        the real session: Left's white body washed out against the bright plywood, the tracker's circle 10 cm off on
        empty plywood; Nick: "the blinking LEDs should have caught it"). First where its wheels put it (a robot
        outline there: detect_after_move; not while exploring, its own frame is not the board's yet), then its lights
        (no turn on its spot). Found: the tracker follows it there. Returns (x, y) or None. Logged (auto_finds)."""
        if getattr(getattr(self, "a", None), "no_auto_find", False) or getattr(self, "feed", None) is None or \
                getattr(self, "cam", None) is None or robot in self.held_out:
            return None
        hist = [t for t in (getattr(self, "auto_find_t", None) or {}).get(robot, []) if t > time.time() -
                self.AUTO_FIND_WINDOW_S]
        if len(hist) >= self.AUTO_FIND_MAX:
            self.say("    %s: found again on its own %d times in %.0f min: Nick is asked" % (
                robot, len(hist), self.AUTO_FIND_WINDOW_S / 60), quiet=True)
            return None
        self.__dict__.setdefault("auto_find_t", {})[robot] = hist + [time.time()]
        t0 = time.time()
        w0 = self.tracker.where() if self.tracker is not None and self.tracker.active == robot else None
        how, q = None, None
        try:
            self.link.send_name(robot, "stop")
        except Exception:
            pass
        if self.board is not None and not getattr(self, "exploring", False):
            try:
                q = self.detect_after_move(robot)
                how = "where its wheels put it" if q is not None else None
            except Exception as e:
                self.say("    (auto find, wheels: %s)" % e, quiet=True)
        if q is None:
            try:
                q = self.identify(robot, lights_only=True)
                how = "its lights" if q is not None else None
            except Exception as e:
                self.say("    (auto find, lights: %s)" % e, quiet=True)
        if q is not None and self.tracker is not None and how == "its lights":
            self.tracker.follow(robot, q[0], q[1])
            self.history.clear()
            self.tracker.hold(HOLD_CM)
            self.__dict__.setdefault("settle_until", {})[robot] = time.time() + FOUND_SETTLE_S
        jump = None if (q is None or w0 is None) else math.hypot(q[0] - w0[1], q[1] - w0[2])
        rec = [round(t0, 1), robot, why, how, None if q is None else [round(q[0], 1), round(q[1], 1)],
               None if jump is None else round(jump, 1), round(time.time() - t0, 1)]
        self.st.setdefault("auto_finds", []).append(rec)
        if q is not None:
            self.__dict__.setdefault("auto_found", {})[robot] = (time.time(), q, how)
            self.say("  %s: %s: found again by %s at (%.1f, %.1f)%s, without asking." % (
                robot, why, how, q[0], q[1], "" if jump is None else ", %.0f cm from the camera's old spot" % jump))
            self.mark("e02 auto find %s" % robot)
        else:
            self.say("  %s: %s: not found again on its own (lights, wheels): Nick is asked." % (robot, why))
        return q

    def recently_auto_found(self, robot, within_s=10.0):
        af = (getattr(self, "auto_found", None) or {}).get(robot)
        return af if af is not None and time.time() - af[0] <= within_s else None

    # --- a hung or stuck robot is stopped at once (2026-10-26 runner fixes, item 4) -----------------------------
    def stuck_why(self, robot, now):
        """Words if the robot's wheels are driving a go leg (crawls included) and the camera, which sees it, has seen
        it move under STUCK_CAMERA_CM in STUCK_MIN_S while the wheels should have taken it STUCK_WHEELS_CM, else ''.
        (21:45:05-35: ray 8's wheels counted 104.85 cm, the camera saw 1 px, for 30 s.)"""
        if self.leg_kind.get(robot) != "go" or robot not in self.leg_live:
            return ""
        t_leg = Runner.leg_started_at(self, robot, now)
        if t_leg is None or now - t_leg < STUCK_MIN_S:
            return ""
        last_t = self.history.last_t() if hasattr(self.history, "last_t") else None
        if last_t is None or now - last_t > 1.0:
            return ""                                    # the camera does not see it: the lost check's job
        cms = self.speed_now.get(robot) or EXPLORE_FAST_CMS
        # only while it must still be driving for the whole window, by its own leg's length and speed (dry run D4: a
        # 12 cm crawl that had ended 2 s before, its end record not in yet, read as "stuck"; D2: an end record lost)
        drive_end = t_leg + LOST_START_S + abs(self.leg_val.get(robot) or 0.0) / max(0.5, cms)
        if now - STUCK_MIN_S < t_leg + LOST_START_S or now > drive_end:
            return ""
        expect = cms * STUCK_MIN_S                       # what the wheels drive in the window
        if expect < STUCK_WHEELS_CM:
            return ""
        smp = self.history.since(now - STUCK_MIN_S)
        if len(smp) < 6:
            return ""
        x0, y0 = sorted(q[1] for q in smp[:3])[1], sorted(q[2] for q in smp[:3])[1]
        x1, y1 = sorted(q[1] for q in smp[-3:])[1], sorted(q[2] for q in smp[-3:])[1]
        moved = math.hypot(x1 - x0, y1 - y0)
        if moved >= STUCK_CAMERA_CM:
            return ""
        return ("its wheels should have taken it about %.0f cm in the last %.0f s but the camera saw it move %.1f cm" % (
            expect, STUCK_MIN_S, moved))

    def tilt_why(self, robot, now=None):
        """Words if the robot's own tilt sensor (RAW pitch / roll, while it streams them) has been more than
        TILT_STOP_DEG from its own level for TILT_MS, else ''. Firmware 2026-10-26a stops itself too (motion.h TILT)."""
        lis = getattr(self, "listener", None)
        if lis is None or not hasattr(lis, "tilt_dev"):
            return ""
        dev = lis.tilt_dev(robot, TILT_MS / 1000.0, now)
        if dev is None or dev <= TILT_STOP_DEG:
            return ""
        return "its own tilt sensor reads %.1f deg off level (limit %.0f): hung at an edge, or a wheel off" % (
            dev, TILT_STOP_DEG)

    def hung_stop(self, robot, kind, why):
        """Stop the robot at once (3 stops), then: a stuck robot is first looked for by its lights (the camera may be
        on something else: then it is not stuck, the tracker follows it and nothing else happens); a confirmed stuck
        or a tilted robot holds the session for Nick (kind 'stuck' / 'tilt'). Returns the hold's answer or 'found'."""
        for _ in range(3):
            try:
                self.link.send_name(robot, "stop")
            except Exception:
                pass
            time.sleep(0.05)
        self.watchdog_fired = True
        self.watch_stop = "%s: %s" % (kind, why)
        self.st.setdefault("hung_stops", []).append([round(time.time(), 1), robot, kind, why, self.phase])
        self.say("!!! %s STOPPED (%s): %s" % (robot, kind, why))
        self.mark("e02 %s stop %s" % (kind, robot))
        if kind == "stuck":
            lid_ = (getattr(self, "last_lid", None) or {}).get(robot)
            m_ = (self.moves.get(robot) or {}).get("run", "-")
            w0 = self.tracker.where() if self.tracker is not None and self.tracker.active == robot else None
            q = self.auto_find(robot, "wheels turning, camera sees no move")
            fin = self.listener.leg_final(robot, m_, lid_) if lid_ is not None and self.listener is not None else None
            if fin is not None and "stop command" not in (fin.get("info") or ""):
                # its own end record came in meanwhile: it had finished the leg, it was not stuck
                self.say("  %s: its leg had ended (%s): not stuck." % (robot, fin.get("state")))
                self.st["hung_stops"][-1].append("leg had ended")
                return "found"
            if q is not None and w0 is not None and math.hypot(q[0] - w0[1], q[1] - w0[2]) > self.AUTO_FIND_SAME_CM:
                self.say("  %s: its lights are %.0f cm from where the camera was: the camera had lost it, it is not "
                         "stuck." % (robot, math.hypot(q[0] - w0[1], q[1] - w0[2])))
                self.st["hung_stops"][-1].append("camera was off")
                return "found"
            words = why + ("; its lights confirm it has not moved" if q is not None else "")
        else:
            words = why
        return self.hold_for_nick(robot, kind, words)

    def self_rescue(self, robot, kind, why):
        """Item 7 (Nick, 2026-10-26: "get as much data as we can before ending ... OK with pausing them and watching
        others, or a better rescue mode"): a robot's hold that nobody answered by its deadline. The robot tries to get
        itself back:
          - at an open edge (a border hold, a stuck robot, or the camera / its own edge stop says so): straight out
            (no turn on its spot: edge_clear_before_turn; else its last forward drive reversed: back_out);
          - lost (or a find that stopped): found by its lights, then, if the camera still can't see it, it finds
            itself against the boards (rescue_find) and the camera looks there;
          - tilted: nothing drives it (a hung robot driven may fall): it fails at once.
        True if it is back (the caller carries on as after Nick's "carry on"); False: the caller leaves it out.
        Every attempt is logged (state "rescues", the video marks)."""
        t0 = time.time()
        steps = []
        ok = False
        self.mark("e02 rescue %s %s" % (robot, kind))
        self.say("  RESCUE: nobody answered the hold for %s (%s): it tries to get itself back." % (robot, why))
        try:
            if kind == "tilt":
                steps.append("tilted: not driven (a hung robot driven may fall)")
            else:
                edge = kind in ("border", "stuck") or bool(Runner.at_edge_why(self, robot))
                if edge:
                    if (getattr(self, "edge_stop_t", None) or {}).get(robot) is not None or \
                            (getattr(self, "last_go", None) or {}).get(robot):
                        good = self.edge_clear_before_turn(robot, explore=bool(getattr(self, "exploring", False)))
                        steps.append("straight out from the edge: %s" % ("clear" if good else "not clear"))
                    else:
                        good = self.back_out(robot, within_s=900.0)
                        steps.append("its last forward drive reversed: %s" % ("done" if good else "no drive known"))
                    room = self.camera_edge_room(robot)
                    ok = bool(good) and (room is None or room >= EDGE_TURN_CM - 0.5)
                    if room is not None:
                        steps.append("the camera: %.1f cm from the edge" % room)
                    if kind == "stuck" and ok:
                        w = self.tracker.where() if self.tracker is not None else None
                        steps.append("the camera sees it %s" % ("again" if w else "nowhere"))
                if kind in ("lost", "find") or (not ok and kind != "border"):
                    q = self.auto_find(robot, "rescue: " + why)
                    steps.append("its lights: %s" % ("found at (%.1f, %.1f)" % q if q else "not found"))
                    if q is None and self.board is not None and not getattr(self, "exploring", False):
                        q = self.rescue_find(robot, need_camera=False)
                        steps.append("its own find against the boards: %s" % (
                            "at (%.1f, %.1f)" % q if q else "failed"))
                        if q is not None:
                            self.st["positions"][robot] = [q[0], q[1], None]
                    ok = q is not None
        except KeyboardInterrupt:
            raise
        except Exception as e:
            steps.append("error: %s" % e)
            ok = False
        rec = [round(t0, 1), robot, kind, why, ok, steps, round(time.time() - t0, 1)]
        self.st.setdefault("rescues", []).append(rec)
        self.mark("e02 rescue %s %s" % (robot, "ok" if ok else "failed"))
        self.say("  RESCUE %s: %s (%s)." % (robot, "it is back" if ok else "failed: it is left out, the others carry on",
                                            "; ".join(steps) or "-"))
        if not ok:
            self.alert("left_out", robot, "%s could not rescue itself (%s): left out" % (robot, kind))
        try:
            self.st.save()
        except Exception:
            pass
        return ok

    def hold_for_nick(self, robot, kind, why, detail=None):
        """Stop every robot and wait for Nick's answer (the live page, or the console): no timeout carries on by
        itself (Nick: "pause and wait for my guidance so we don't lose all 3 robots to the same problem"). kind is
        'border' or 'lost'. Returns his answer: 'c' carry on, 'a' carry on and find lost robots again without
        asking from now on, 'o' leave this robot out (it stays where it is; the others carry on), 's' stop the
        session (sticky: every move after it raises KeyboardInterrupt in the main thread; --resume later).
        2026-10-24 review: nothing drives during a hold (an "o" only records where the robot was last seen), the
        stop is sent again every HOLD_RESTOP_S while it waits, and a question asked by another part of the runner
        waits for the hold's answer instead of replacing it on the page.
        kind 'find' (a backing-in find found no board; find_hold): detail says where it is and what to check (the
        live page shows it under the question); 'c' tries the find again after a fresh camera fix, 'a' means "the
        board is gone" (the edge find; it does not change how lost robots are handled)."""
        if self.stop_requested:
            self.raise_if_stopped()
            return "s"
        if robot in self.held_out:
            return "o"
        if kind == "lost" and robot is not None and not self.hold.is_set():
            # coordinator, 2026-10-26: FIRST find it again on its own (lights, its wheels' expected spot); ask Nick
            # only if that fails. Nothing else moves meanwhile (the hold flag is up while it looks)
            got = None
            with self.hold_lock:
                self.hold.set()
                self.hold_owner = threading.current_thread()
                try:
                    self.stop_everyone(rounds=1)
                    got = self.auto_find(robot, why)
                finally:
                    self.hold_owner = None
                    self.last_hold_end = time.time()
                    self.hold.clear()
            if got is not None:
                self.st.setdefault("holds", []).append([round(time.time(), 1), robot, kind, why, self.order, "found",
                                                        0.0, "found again on its own"])
                return "c"
        if kind == "lost" and self.hold_auto_refind:
            # Nick's "a": no question, but the robot is still stopped and found again (the caller re-finds it)
            try:
                self.link.send_name(robot, "stop")
            except Exception:
                pass
            self.say("  %s: %s (stopped; finding it again without asking, as Nick chose)" % (robot, why), quiet=True)
            return "c"
        ans = "c"
        t_call = time.time()
        timed_out_here = False
        with self.hold_lock:
            if self.stop_requested:
                self.raise_if_stopped()
                return "s"
            if self.last_hold_end > t_call or robot in self.held_out:
                # a hold ended while this one waited: everyone was stopped and Nick answered since this was seen
                return self.hold_answer or "c"
            self.hold.set()
            if self.tracker is not None:
                self.tracker.in_hold = True
            self.hold_owner = threading.current_thread()
            self.hold_answer = None
            restop = threading.Event()

            def keep_stopping():
                while not restop.wait(HOLD_RESTOP_S):
                    self.stop_everyone(rounds=1)
            try:
                self.stop_everyone()
                threading.Thread(target=keep_stopping, daemon=True).start()
                rec = [round(time.time(), 1), robot, kind, why, self.order, None, None, detail or ""]
                try:
                    self.mark("e02 hold %s %s: %s" % (kind, robot, why))
                    self.st.setdefault("holds", []).append(rec)
                    self.alert("hold", robot, why)          # the live page sounds (15:45 fixes)
                    self.st.save()
                except Exception as e:              # the bookkeeping must never skip the question
                    self.say("  (hold: could not record it: %s)" % e, quiet=True)
                t0 = time.time()
                t_sess, _ = self.session_deadline(t0, None)
                print("\n" + "!" * 78)
                self.say("HOLD: every robot is stopped. %s: %s." % (robot, why))
                if detail:
                    self.say(detail)
                self.say("The session waits for your answer%s." % (
                    " (no time limit)" if t_sess is None else ": " + self.end_page_words(t_sess).replace(
                        "No answer", "no answer")))
                print("!" * 78)
                if kind == "find":
                    prompt = ("HOLD (find stopped): %s: %s. Every robot is stopped. Answer: c = carry on (it tries the "
                              "find again after a fresh camera fix), a = the board is gone (it finds itself against "
                              "any two edges instead), o = leave %s out (it stays where it is), s = stop the session "
                              "(--resume later): " % (robot, why, robot))
                else:
                    prompt = ("HOLD (%s): %s: %s. Every robot is stopped. Answer: c = carry on, a = carry on and find "
                              "lost robots again without asking from now on, o = leave %s out (it stays where it is), "
                              "s = stop the session (--resume later): " % (
                                  dict(border="virtual border", tilt="robot tilted", stuck="robot stuck",
                                       link="radio link").get(kind, "lost robot"), robot, why, robot))
                rescued = None
                while True:
                    a_ = self.hold_ask(prompt, t_asked=t0, rescue_ok=robot is not None and kind in RESCUE_KINDS)
                    if a_ == Runner.RESCUE:
                        # item 7 (Nick, 2026-10-26: "get as much data as we can before ending"): nobody answered by
                        # the deadline: the robot rescues itself; if it can't, it is left out and the others carry on
                        restop.set()                         # (the rescue's own moves must not be stopped)
                        rescued = self.self_rescue(robot, kind, why)
                        ans = "c" if rescued else "o"
                        break
                    if self.timed_out:
                        ans, timed_out_here = "s", True     # nobody answered in time: the session ends
                        break
                    a_ = (a_ or "").strip().lower()[:1]
                    if a_ in ("c", "a", "o", "s"):
                        ans = a_
                        break
                    self.say("  (answer c, a, o or s)", quiet=True)
                    time.sleep(1.0)
                rec[5], rec[6] = ("timeout" if timed_out_here else ("rescued" if rescued else "rescue_failed")
                                  if rescued is not None else ans), round(time.time() - t0, 1)
                if rescued is not None:
                    self.say("  No answer by the deadline: %s %s." % (robot, "rescued itself: the session carries on"
                                                                      if rescued else "could not rescue itself: it is "
                                                                      "left out and the others carry on"))
                elif timed_out_here:
                    self.say("  No answer: the session ends (%s)." % ((self.end_reason or {}).get("why") or "timeout"))
                else:
                    self.say("  Nick's answer: %s" % dict(
                    c="carry on" + (": the find again after a fresh camera fix" if kind == "find" else ""),
                    a="the board is gone: the edge find" if kind == "find" else
                    "carry on; lost robots found again without asking from now on", o="leave %s out" % robot,
                    s="stop the session")[ans])
                if ans == "o" and rescued is not None and not self.leave_out(robot):
                    # a failed rescue of a robot nobody can place: it is left out all the same, kept where it was
                    # last known if anywhere; the others keep RESCUE_CLEAR_CM from that place (keep_on_map)
                    self.held_out.add(robot)
                    self.unavailable.add(robot)
                    self.moves.pop(robot, None)
                    self.st.setdefault("left_out", []).append([round(time.time(), 1), robot, None])
                elif ans == "o" and not self.leave_out(robot):
                    self.say("  Nobody knows where %s is, so the others could drive into it: the session stops "
                             "instead (--resume later)." % robot)
                    ans = "s"
                if ans == "o" and rescued is not None and not [n for n in self.st["robots"]
                                                              if n not in self.unavailable and n not in self.held_out]:
                    self.hold_answer = "s"
                    restop.set()
                    self.session_timeout(TO.NO_ROBOTS, t0, why)
                    ans = "s"
                if ans == "a" and kind != "find":
                    self.hold_auto_refind = True
                if ans == "s":
                    self.stop_requested = True
                self.hold_answer = ans
                try:
                    self.mark("e02 hold %s %s" % ("timeout" if timed_out_here else "answer " + ans, robot))
                    self.st.save()
                except Exception:
                    pass
            finally:
                restop.set()
                if self.tracker is not None:
                    self.tracker.in_hold = False
                self.hold_owner = None
                self.last_hold_end = time.time()
                self.hold.clear()
        if ans == "s":
            self.raise_if_stopped()
        return ans

    def raise_if_stopped(self):
        """Nick answered s at a hold: in the main thread, end the session the Ctrl+C way."""
        if self.stop_requested and threading.current_thread() is threading.main_thread():
            raise KeyboardInterrupt

    def leave_out(self, robot):
        """Nick's "o": the robot does not move again this session; it stays on the map where it was last seen (the
        tracker, else its last parked place) so the others go round it. Nothing drives (2026-10-24 review: the old
        way searched for it with another robot, during the hold). False if nobody knows where it is."""
        q = None
        if self.tracker is not None and self.tracker.active == robot:
            w = self.tracker.where()
            if w:
                q = (w[1], w[2])
        if q is None and self.st["positions"].get(robot):
            q = tuple(self.st["positions"][robot][:2])
        if q is None:
            return False
        self.held_out.add(robot)
        self.unavailable.add(robot)
        self.moves.pop(robot, None)
        self.st["positions"][robot] = [q[0], q[1], None]
        if self.tracker is not None:
            try:
                self.tracker.park(robot, q[0], q[1])
            except Exception:
                pass
        self.st.setdefault("left_out", []).append([round(time.time(), 1), robot, [round(q[0], 1), round(q[1], 1)]])
        return True

    def hold_ask(self, prompt, t_asked=None, rescue_ok=False):
        """Nick's answer: the live page (and the console) when there is one, else the console; a dry run with
        nobody there answers c after HOLD_DRY_ANSWER_S. With no page and no console (started without a terminal),
        nobody can ever answer: the session stops (s). The session timeouts (2026-10-26) end the session if
        nobody answers by the hold's deadline (t_asked: the hold's start): self.timed_out is then set.
        A --dry-scene hold answered "none": nobody answers it (the timeouts' simulator test)."""
        nxt = getattr(self, "dry_next_answer", None)
        if nxt == "none" and getattr(self.a, "dry_run", False):
            nxt = None                              # left unanswered: the page (or the console) waits as for real
        elif nxt and getattr(self.a, "dry_run", False):
            self.dry_next_answer = None            # --dry-scene: this hold's own answer
            time.sleep(HOLD_DRY_ANSWER_S)
            self.say(prompt + " -> " + nxt + " (dry run: the scene's answer)")
            return nxt
        if getattr(self.a, "dry_run", False) and (self.a.auto or not getattr(self.a, "web_ask", False)) \
                and getattr(self, "dry_next_answer", None) != "none":
            time.sleep(HOLD_DRY_ANSWER_S)
            self.say(prompt + " -> " + getattr(self.a, "dry_hold_answer", "c") + " (dry run: automatic)")
            return getattr(self.a, "dry_hold_answer", "c")
        if getattr(self, "view", None) is not None:
            ans = self.web_ask(prompt, "", None, hold=True, t_asked=t_asked, rescue_ok=rescue_ok)
            if ans or self.timed_out:
                return ans
            time.sleep(1.0)
            return ""
        if sys.stdin is None or not sys.stdin.isatty():
            self.say("  (no live page and no console to answer on: the session stops)")
            return "s"
        return self.console_wait(prompt, t_asked=t_asked, rescue_ok=rescue_ok) or ""

    def make_guard(self):
        """The border while exploring (the robots' own camera fit and board are not made yet): the calibration file
        (--calib, else the session-1 roam1 file) and the board laid out as --board-sides says. Positions on the
        provisional camera go back to pixels and onto this one. None (said loudly) if there is no file."""
        cal = getattr(self.a, "calib", None) or (DEFAULT_CALIB if os.path.exists(DEFAULT_CALIB) else None)
        sim_cal = os.path.join(self.dir, "sim", "camera_model.json")
        if getattr(self.a, "dry_run", False) and os.path.exists(sim_cal):
            cal = sim_cal                  # a dry run: the simulator's camera stands in for the session-1 file
        if not cal or not os.path.exists(cal):
            self.say("  WARNING: no calibration file for the virtual border while exploring: explore runs without it.")
            return None
        try:
            cam = CAM.load_camera(cal)
        except Exception as e:
            self.say("  WARNING: the virtual border can't use %s (%s): explore runs without it." % (cal, e))
            return None
        xs, ys = (int(v) for v in (getattr(self.a, "board_sides", None) or "1,1").split(","))
        self.say("  Virtual border while exploring: %.0f cm inside the open edges, placed by %s (until explore's own "
                 "fit)." % (self.border_cm(), os.path.basename(cal)))
        sh = getattr(self.a, "dry_guard_shift", "") if getattr(self.a, "dry_run", False) else ""
        if sh:
            # (the dark strips at the bottom and right edges are painter's tape, surface: the real drop is near the
            # stored bottom line and about 4 cm outside the stored right line,
            # 2026-10-25 15:45 analysis; the shift re-creates where the 15:10 border's line sat, not a drop)
            # the 15:10 re-creation: the stored outline off the plywood, so a line-sensor edge stop leaves the
            # robot's centre past the border's line (as 3.8-5.5 cm past at every 15:10 edge stop)
            dx, dy = (float(v) for v in sh.split("/"))
            cam = ShiftedFloor(cam, dx, dy)
            self.say("  (dry run: the border's calibration shifted %+.0f / %+.0f cm off the simulator's floor)" % (dx, dy))
        return cam, PL.Board(xboard_side=xs, yboard_side=ys)

    def border_frame(self, x, y):
        """A tracker position as (board, x, y) for the border: the session's own board, or the guard while
        exploring. None if neither."""
        if self.board is not None and not getattr(self, "exploring", False):
            return self.board, x, y
        if self.guard is None or self.cam is None:
            return None
        u, v = self.cam.project((x, y, EX.ROBOT_TOP_CM))
        gx, gy = self.guard[0].to_floor(float(u), float(v), EX.ROBOT_TOP_CM)
        if not (np.isfinite(gx) and np.isfinite(gy)):
            return None
        return self.guard[1], float(gx), float(gy)

    def border_cm(self):
        """The virtual border now: BORDER_CM, moved in while the camera looks bad (BORDER_EXTRA_SIGMA)."""
        base = HOME_BORDER_CM if getattr(getattr(self, "a", None), "home", False) else BORDER_CM
        tr = getattr(self, "trust", None)
        if tr is None or not tr.bad("camera"):
            return base
        k = max(1.0, tr.k("camera"))
        extra = BORDER_EXTRA_SIGMA * TR.HEALTHY["camera"] * math.sqrt(k * k - 1.0)
        return base + min(BORDER_EXTRA_MAX_CM, extra)

    def in_border_band(self, robot):
        """Is the robot's latest camera position within BORDER_CM of an open edge (False if not known, or if the
        camera follows another robot)?"""
        if self.tracker is None or self.tracker.active != robot:
            return False
        smp = self.history.since(time.time() - 1.0) if self.history is not None else []
        if not smp:
            return False
        f = self.border_frame(smp[-1][1], smp[-1][2])
        if f is None:
            return False
        bd, x, y = f
        # signed: a centre past the line is inside the band too, however far past (2026-10-25 runner fixes)
        return min(ST.inward_cm(bd, ax, v, x if ax == "x" else y) for ax, v in bd.open_edges()) < self.border_cm()

    SHADOW_EVERY_S = 5.0      # chosen: one shadow record per robot per 5 s at most (one approach, not 50 pictures of it)

    def border_shadow(self, robot, smp, home_tripped):
        """Home sessions (Nick, 2026-10-25): where the AWAY border (BORDER_CM, and the bad-camera shrink) would have
        stopped the session, recorded without stopping anything (border_shadow.csv, state "border_shadow"), with
        whether home's own border tripped too. Over a few sessions: how often the away border fires for a robot that
        really goes on to the edge, against how often for nothing, the evidence for dropping it."""
        last = getattr(self, "shadow_last", None)
        if last is None:
            last = self.shadow_last = {}
        if time.time() - last.get(robot, 0.0) < self.SHADOW_EVERY_S:
            return
        tr = getattr(self, "trust", None)
        away = BORDER_CM
        if tr is not None and tr.bad("camera"):
            away = self.border_cm() - HOME_BORDER_CM + BORDER_CM
        why = self.border_check(robot, smp, bcm=away)
        if not why:
            return
        last[robot] = time.time()
        f = self.border_frame(smp[-1][1], smp[-1][2])
        d = None
        if f is not None:
            bd_, x_, y_ = f
            d = min(ST.inward_cm(bd_, ax, v, x_ if ax == "x" else y_) for ax, v in bd_.open_edges())   # - : past
        rec = [round(time.time(), 2), robot, None if d is None else round(d, 1), round(away, 1), int(home_tripped),
               self.phase, self.order]
        self.st.setdefault("border_shadow", []).append(rec)
        path = os.path.join(self.dir, "border_shadow.csv")
        new = not os.path.exists(path)
        try:
            with open(path, "a", newline="") as fh:
                w = csv.writer(fh)
                if new:
                    w.writerow(["t", "robot", "edge_cm", "away_border_cm", "home_tripped", "phase", "order"])
                w.writerow(rec)
        except OSError:
            pass
        self.say("    (away border would have stopped the session here: %s; home's %s)" % (
            why, "tripped too" if home_tripped else "did not"), quiet=True)

    STILL_WINDOW_S = 3.0      # chosen: a still robot's camera scatter is measured over this long ...
    STILL_EVERY_S = 10.0      # ... at most this often (one evidence item per axis each time)

    def forward_explore_leg(self, robot):
        """Is the robot exploring, on a forward go leg (its edge guard on)?"""
        return bool(getattr(self, "exploring", False) and self.leg_kind.get(robot) == "go"
                    and (self.leg_val.get(robot) or 0.0) > 0)

    def camera_still_check(self, robot, last_still):
        """The camera's trust from a robot standing still (e02_trust camera_still): nothing sent to it for
        STILL_WINDOW_S + 0.5 s, no run, and the camera sees it the whole window. A find (`run find`, a firmware run
        that drives the whole time) counts as moving until STILL_WINDOW_S + 0.5 s after it ends (2026-10-25 simulator,
        front@Left:noise=6: each setup find was taken for a still robot, 4.8-7.2 cm of "scatter", and the camera was
        flagged bad 3 times). Other firmware runs are not skipped: their settle pauses are real still moments, and
        skipping every open run left no still evidence after setup (2026-10-25 rerun)."""
        now = time.time()
        t_find = (getattr(self, "find_live", None) or {}).get(robot)
        if t_find is not None and now - t_find < self.STILL_WINDOW_S + 0.5:
            return
        if now - last_still.get(robot, 0.0) < self.STILL_EVERY_S:
            return
        if robot in self.leg_live or (self.moves.get(robot) or {}).get("run", "-") not in ("-", None):
            return
        if now - self.leg_end.get(robot, 0.0) < self.STILL_WINDOW_S + 0.5:
            return
        smp = self.history.since(now - self.STILL_WINDOW_S)
        if len(smp) < 15 or smp[-1][0] - smp[0][0] < 0.8 * self.STILL_WINDOW_S:
            return
        xs, ys = np.array([q[1] for q in smp]), np.array([q[2] for q in smp])
        last_still[robot] = now
        self.trust.camera_still(robot, float(np.std(xs)), float(np.std(ys)))
        # flags re-checked with every new piece of evidence (Nick, 2026-10-25: a check costs well under a ms, and
        # how often it runs doesn't change false flags, only how late a real one is seen)
        self.trust_note(save=False)

    def camera_lag_s(self):
        """The camera's lag now (s): the newest perf row's picture age + one picture interval; a camera loop that
        has written no row for over a second (stuck) counts that time too. None before the first row."""
        rows = getattr(self, "perf_rows", None)
        if not rows:
            return None
        r = rows[-1]
        lag = float(r["age_mean_s"]) + 1.0 / max(0.5, float(r["fps_handled"]))
        return max(lag, time.time() - float(r["t"]) - 1.0)

    def speed_guard_cm(self, robot, v_cms=0.0):
        """How much further out this robot's border sits because the camera is slow: its speed (driving, or what it
        will drive at) x the lag above the budget, at most SPEED_GUARD_MAX_CM; 0 within budget."""
        lag = self.camera_lag_s()
        if lag is None:
            return 0.0
        budget = SPEED_LAG_BUDGET_HOME_S if getattr(getattr(self, "a", None), "home", False) else \
            SPEED_LAG_BUDGET_AWAY_S
        v = max(float(v_cms or 0.0), float((getattr(self, "speed_now", {}) or {}).get(robot) or EXPLORE_FAST_CMS))
        return min(SPEED_GUARD_MAX_CM, v * max(0.0, lag - budget))

    def speed_guard_check(self, robot, smp, last):
        """The watchdog's speed guard (part 2b): the camera is slow and the robot drives towards an open edge inside
        its widened border (border_cm + speed_guard_cm): stop it, no hold. True if it was stopped."""
        if len(smp) < 2:
            return False
        dt_ = max(1e-3, smp[-1][0] - smp[0][0])
        v_meas = math.hypot(smp[-1][1] - smp[0][1], smp[-1][2] - smp[0][2]) / dt_
        extra = self.speed_guard_cm(robot, v_meas if v_meas < BORDER_JUMP_CMS else 0.0)
        if extra < SPEED_GUARD_MIN_CM:
            return False
        why = self.border_check(robot, smp, bcm=self.border_cm() + extra)
        if not why or time.time() - last.get(robot, 0.0) < 2.0:
            return bool(why)
        last[robot] = time.time()
        for _ in range(3):
            self.link.send_name(robot, "stop")
            time.sleep(0.1)
        lag = self.camera_lag_s() or 0.0
        rec = [round(time.time(), 2), robot, round(extra, 1), round(lag, 3), why, self.phase, self.order]
        self.st.setdefault("speed_guard_stops", []).append(rec)
        self.say("  SPEED GUARD: %s stopped: the camera is slow (lag %.2f s), so its border is %.1f cm wider: %s. "
                 "It waits until the camera catches up." % (robot, lag, extra, why))
        self.mark("e02 speed guard stop %s" % robot)
        return True

    def in_band(self, robot, bcm):
        """Is the robot's centre (the camera's latest, else its parked spot) within bcm of an open edge (or past
        it)? False if not known."""
        q = None
        if self.tracker is not None and self.tracker.active == robot and self.history is not None:
            smp = self.history.since(time.time() - 1.0)
            if smp:
                q = (smp[-1][1], smp[-1][2])
        if q is None and (self.st.get("positions") or {}).get(robot):
            q = tuple(self.st["positions"][robot][:2])
        if q is None:
            return False
        f = self.border_frame(q[0], q[1])
        if f is None:
            return False
        bd, x, y = f
        return min(ST.inward_cm(bd, ax, v, x if ax == "x" else y) for ax, v in bd.open_edges()) < bcm

    def speed_guard_wait(self, robot, cmd=""):
        """Before a move (main thread): while the camera is slow and the robot is in its widened band, wait until
        the lag is back within budget for SPEED_RECOVER_ROWS rows, at most SPEED_WAIT_MAX_S."""
        extra = self.speed_guard_cm(robot)
        if extra < SPEED_GUARD_MIN_CM or not self.in_band(robot, self.border_cm() + extra):
            return 0.0
        t0 = time.time()
        lag0 = self.camera_lag_s() or 0.0
        self.say("  SPEED GUARD: the camera is slow (lag %.2f s): %s is within %.0f cm of an open edge, so `%s` waits "
                 "until it catches up (at most %.0f s)" % (lag0, robot, self.border_cm() + extra, cmd.strip(),
                                                           SPEED_WAIT_MAX_S))
        good, seen = 0, None
        while time.time() - t0 < SPEED_WAIT_MAX_S and not self.stop_requested:
            time.sleep(0.25)
            rows = getattr(self, "perf_rows", None) or []
            if rows and rows[-1] is not seen:
                seen = rows[-1]
                good = good + 1 if self.speed_guard_cm(robot) < SPEED_GUARD_MIN_CM else 0
                if good >= SPEED_RECOVER_ROWS:
                    break
        waited = time.time() - t0
        self.st.setdefault("speed_guard_waits", []).append([round(t0, 1), robot, round(extra, 1), round(lag0, 3),
                                                            round(waited, 1), good >= SPEED_RECOVER_ROWS])
        self.say("  SPEED GUARD: %s after %.0f s" % ("camera caught up: going on" if good >= SPEED_RECOVER_ROWS else
                                                     "still slow: the move goes, the in-motion guard still on", waited),
                 quiet=good >= SPEED_RECOVER_ROWS)
        return waited

    # --- the radio link (2026-10-26 runner fixes, item 1d / 1e; e02_link.py) -----------------------------------
    LINK_TICK_S = 1.0             # chosen: the gauge is looked at once a second (no DOE: display / reaction pace)
    link_paused = None            # when new moves started waiting for the link (None: they don't)
    link_level = None

    def link_tick(self, now=None):
        """Once a second (the watchdog's loop): the link gauge's level. 'warn' is said; 'pause' makes new moves wait
        (link_gate) until the last minute is back under the warning level. Kept in the state (link, link_events)."""
        lk = getattr(self, "link", None)
        if lk is None or not hasattr(lk, "gauge"):
            return
        now = time.time() if now is None else now
        if now - getattr(self, "_link_t", 0.0) < self.LINK_TICK_S:
            return
        self._link_t = now
        st_ = lk.status(now=now)
        self.st["link"] = st_
        lvl = st_["level"]
        pct = None if st_["share"] is None else 100.0 * st_["share"]
        ev = self.st.setdefault("link_events", [])
        if lvl == "pause" and self.link_paused is None:
            self.link_paused = now
            words = ("the radio link needs a resend for %.0f %% of commands over 5 min (clearly above the %.0f %% pause "
                     "level): new moves wait until the last 2 minutes are under it" % (pct, 100 * LNK.LINK_PAUSE))
            self.say("!!! LINK PAUSE: " + words)
            self.mark("e02 link pause")
            self.alert("link", None, words)
            ev.append([round(now, 1), "pause", round(pct, 1)])
        elif self.link_paused is not None and lk.gauge.recovered(now=now):
            s1, _ = lk.gauge.share(LNK.LINK_RESUME_S, now=now, min_n=1)
            self.say("  LINK: back to %.0f %% over the last 2 minutes after %.0f s: moves go on." % (
                100.0 * (s1 or 0.0), now - self.link_paused))
            self.mark("e02 link resume")
            ev.append([round(now, 1), "resume", round(100.0 * (s1 or 0.0), 1), round(now - self.link_paused, 1)])
            self.link_paused = None
        elif lvl == "warn" and self.link_level not in ("warn", "pause") and \
                now - getattr(self, "_link_warned", 0.0) > LNK.LINK_WINDOW_S:
            self._link_warned = now                  # (once per 5 min: dry run D1 said it every few seconds at 5 %)
            self.say("!!! LINK: %.0f %% of commands needed a resend over 5 min (normal sessions: under 5 %%). They "
                     "are resent until they arrive; above %.0f %% new moves wait." % (pct, 100 * LNK.LINK_PAUSE))
            self.mark("e02 link warn")
            ev.append([round(now, 1), "warn", round(pct, 1)])
        if lvl is not None:
            self.link_level = lvl
        if self.link_paused is not None and now - self.link_paused > LINK_PAUSE_MAX_S and LINK_PAUSE_MAX_S > 0 \
                and not getattr(self, "timed_out", False):
            self.session_timeout(TO.LINK, self.link_paused, "the radio link")

    def link_gate(self, robot, cmd=""):
        """Before a move (main thread): while the link is paused (link_tick), wait. Returns the seconds waited."""
        if self.link_paused is None:
            return 0.0
        t0 = time.time()
        self.say("  LINK PAUSE: `%s` to %s waits for the radio link" % (cmd.strip(), robot))
        while self.link_paused is not None and not self.stop_requested and not getattr(self, "timed_out", False):
            time.sleep(0.5)
            if getattr(self, "watch_quit", None) is None:
                self.link_tick()                     # (no watchdog thread: the tests' stand-ins)
        w = time.time() - t0
        self.st.setdefault("link_waits", []).append([round(t0, 1), robot, cmd.strip()[:40], round(w, 1)])
        return w

    def spin_check(self, robot, smp, bcm=None):
        """Fix 5: during a turn on its spot, words if the robot's centre (median of the last 3 pictures) is inside
        the border and has moved more than TURN_SPIN_DRIFT_CM towards an open edge since the turn began, else ''."""
        s0 = (getattr(self, "turn_start_xy", None) or {}).get(robot)
        if s0 is None or len(smp) < 3:
            return ""
        xs, ys = sorted(q[1] for q in smp[-3:])[1], sorted(q[2] for q in smp[-3:])[1]
        a = self.border_frame(s0[0], s0[1])
        b = self.border_frame(xs, ys)
        if a is None or b is None:
            return ""
        bd, x, y = b
        bcm = self.border_cm() if bcm is None else bcm
        for ax, val in bd.open_edges():
            d = ST.inward_cm(bd, ax, val, x if ax == "x" else y)
            d0 = ST.inward_cm(bd, ax, val, a[1] if ax == "x" else a[2])
            if d < bcm and d0 - d > TURN_SPIN_DRIFT_CM:
                return ("its centre moved %.1f cm towards an open edge while it turned on its spot (now %.0f cm from "
                        "it; the border is %.0f cm)" % (d0 - d, d, bcm))
        return ""

    def border_check(self, robot, smp, bcm=None):
        """Words if the robot is within BORDER_CM of an open edge and moving towards it, else ''. smp: the tracker's
        recent (t, x, y, ...) samples."""
        if robot in self.seek or robot == getattr(self, "edge_touch", None):
            return ""
        # after a crawl towards the edge (or the heading drive), only what the camera saw after it stopped counts
        # (2026-10-24 review: a time window exempted the first second of the next leg, backing included)
        t_end = getattr(self, "seek_end", {}).get(robot)
        if t_end is not None:
            smp = [q for q in smp if q[0] > t_end]
        if len(smp) < 3:
            return ""
        driving = robot in self.leg_live or (self.moves.get(robot) or {}).get("run", "-") not in ("-", None)
        if not driving:
            # standing still: the camera's estimate settling after the tracker moves to it is not the robot moving
            # (dry run 2026-10-24 19:41: Center held the session while it stood, 5 cm of settling "towards" the edge)
            return ""
        tr = getattr(self, "trust", None)
        lis = getattr(self, "listener", None)
        if tr is not None and tr.bad("camera") and lis is not None and \
                lis.firmware.get(robot) in BORDER_FW:
            # a camera that looks bad: routes are kept inside the shrunken area (border_cm), and the robot's own
            # border (FW_BORDER_CM, its wheels and gyro) is the line; the camera's would hold on its own noise (2026-10-25 simulator, camera noise 4 cm: 6 camera
            # border holds in 15 minutes, none with the robot really at the edge)
            return ""
        if robot in self.leg_live and self.leg_kind.get(robot) == "turn":
            # turning on its spot: the camera's spot (the top's centre, not the axle) swings a few cm, the robot
            # goes nowhere (dry run 2026-10-24: a 90 deg turn 15 cm from the edge held the session). But a spin that
            # carries the robot towards an open edge inside the border is held like a drive (fix 5; 15:45: Right's
            # 5.4 s spins with its body over the tape strip were exempt)
            return Runner.spin_check(self, robot, smp, bcm)
        if Runner.backing_out(self, robot):
            # 2026-10-26 runner fixes, item 2: backing straight out after its own edge stop is always allowed, here
            # too (21:14: two holds of Left's back-out, "moving further out", on pictures of the crawl before it and on
            # 0.8 cm of wobble; Nick then let the runner turn it at the corner)
            return ""
        # a straight line fitted through the window's pictures, not its first and last single pictures: a still
        # robot's fitted 0.6 s move stays under BORDER_JITTER_CM that way (4 h of the 21:14 overnight hold)
        (x0m, y0m), (x1m, y1m) = line_ends(smp)
        a = self.border_frame(x0m, y0m)
        b = self.border_frame(x1m, y1m)
        if a is None or b is None:
            return ""
        bd, x, y = b
        bcm = self.border_cm() if bcm is None else bcm
        dt_ = max(1e-3, smp[-1][0] - smp[0][0])
        speed = math.hypot(x - a[1], y - a[2]) / dt_
        for ax, val in bd.open_edges():
            # signed, as ST.hazard: + on the board, - past the line (2026-10-25 15:10: with abs(), every back-off
            # from a line-sensor edge stop, centre 3.8-5.5 cm past the stored line, read as approaching: 6 of 9
            # border trips false, 4 holds)
            d = ST.inward_cm(bd, ax, val, x if ax == "x" else y)
            if d >= bcm:
                continue
            d0 = ST.inward_cm(bd, ax, val, a[1] if ax == "x" else a[2])
            if d0 - d < BORDER_JITTER_CM:
                continue                                              # within the camera's wobble: not a move
            towards = (d0 - d) / dt_                                  # cm/s closer to (or further past) that edge
            if BORDER_TOWARDS_CMS < towards < BORDER_JUMP_CMS and towards >= BORDER_TOWARDS_FRAC * speed:
                if d < 0:
                    return ("its centre is %.0f cm past an open edge's line (the border is %.0f cm), moving further "
                            "out" % (-d, bcm))
                return "its centre is %.0f cm from an open edge (the border is %.0f cm), moving towards it" % (
                    d, bcm)
        return ""

    def watchdog(self, robot=None):
        """For wait_end polls: has the camera watchdog stopped the robot in this run?"""
        return self.watchdog_fired

    STILL_GRACE_S = 2.0       # chosen: a robot counts as maybe moving this long after its last leg ended (braking, a
    # late answer). DOE note: no, a margin

    def may_move(self, robot, now=None):
        """Could the robot be moving now? A leg is live, a camera-steered or firmware run is going (RUNINFO without
        its end), a find is running, it is crawling, or its last leg ended under STILL_GRACE_S ago."""
        now = time.time() if now is None else now
        if robot in self.leg_live or robot in self.seek:
            return True
        if now < (getattr(self, "settle_until", None) or {}).get(robot, 0.0):
            return False                  # just found, standing still: the tracker is settling on it
        if (self.moves.get(robot) or {}).get("run", "-") not in ("-", None):
            return True
        if (getattr(self, "find_live", None) or {}).get(robot) == float("inf"):
            return True
        w = getattr(self, "watch", None)
        cur = (getattr(w, "started", None) or {}).get(robot) if w is not None else None
        if cur and cur[0] not in (getattr(w, "ended", None) or {}):
            return True
        return now - self.leg_end.get(robot, 0.0) < self.STILL_GRACE_S

    def backing_out(self, robot, now=None):
        """Fix 4: is the robot on a backwards drive right after its own edge guard stopped it? (15:45: the watchdog
        stopped Left's one right move, backing out, as "4 cm past the open edge and still going".) 2026-10-26 runner
        fixes, item 2: "right after" = no turn and no forward drive has run since the edge stop (refused legs do not
        count), however long a hold took in between (21:14: Left's second back-out came 27 s after its edge stop,
        after a 104 s hold it would have been outside a 30 s window)."""
        t_e = (getattr(self, "edge_stop_t", None) or {}).get(robot)
        return bool(t_e is not None and robot in self.leg_live
                    and self.leg_kind.get(robot) == "go" and (self.leg_val.get(robot) or 0.0) < 0)

    def leg_started_at(self, robot, now=None):
        """When the robot's live leg really began (laptop time): its own 'started' LEG record; else its answer to the
        command (numbered firmware, e02_link) + LEG_START_AFTER_ACK_S; else the send + LOST_NO_START_S (older firmware,
        'started' record lost). None if the leg is not live, or none of these yet (2026-10-26 runner fixes, item 1c:
        at 21:14 both lost holds counted from the send of a back-out the robot never got)."""
        t_send = self.leg_live.get(robot)
        if t_send is None:
            return None
        lid = (getattr(self, "last_lid", None) or {}).get(robot)
        lis = getattr(self, "listener", None)
        if lis is not None and lid is not None and hasattr(lis, "recs"):
            with lis.cv:
                for t, n, rec in reversed(lis.recs["LEG"]):
                    if t < t_send - 0.5:
                        break
                    if n == robot and rec.get("id") == str(lid) and (rec.get("state") or rec.get("result")) == "started":
                        return t
        lk = getattr(self, "link", None)
        txt = (getattr(self, "leg_text", None) or {}).get(robot)
        if lk is not None and txt and hasattr(lk, "ack_time"):
            ta = lk.ack_time(robot, txt)
            if ta is not None and ta >= t_send - 0.5:
                return ta + LEG_START_AFTER_ACK_S
            if hasattr(lk, "numbered") and lk.numbered(robot):
                return None                  # no answer yet: it has not got the command, so it is not driving
        now = time.time() if now is None else now
        return t_send + LOST_NO_START_S if now - t_send > LOST_NO_START_S else None

    def lost_why(self, robot, now):
        """The watchdog's lost check for the followed robot: words if it is lost (a driving leg the camera has not
        seen for LOST_DRIVE_S, or whose spot does not move while its wheels drive), else ''. Counted from when the
        leg really began (leg_started_at), not from the send."""
        t_leg = Runner.leg_started_at(self, robot, now) if robot in self.leg_live else None
        # a robot turning on its spot goes nowhere: the camera gets RECHECK_S to see it again before it is called
        # lost (2026-10-25 15:10: 3 lost holds during turns, camera gaps of 1.8-3.0 s beside a robot or a light
        # object; each hold cut the turn short)
        lost_s = RECHECK_S if self.leg_kind.get(robot) == "turn" else LOST_DRIVE_S
        if t_leg is None or robot in self.refinding or robot in self.seek or now - t_leg <= lost_s:
            return ""
        last_t = self.history.last_t() if hasattr(self.history, "last_t") else None
        if last_t is None or now - max(last_t, t_leg) > lost_s:
            return "the camera has not seen it for %.1f s while it %s" % (
                now - max(last_t or t_leg, t_leg), "turns" if lost_s == RECHECK_S else "drives")
        if self.leg_kind.get(robot) == "go":
            since = self.history.since(t_leg)
            el = now - t_leg
            cms = self.speed_now.get(robot) or EXPLORE_FAST_CMS
            expect = min(abs(self.leg_val.get(robot) or 0.0), cms * max(0.0, el - LOST_START_S))
            if el > leg_over_s(self.leg_val.get(robot), cms):
                return ""        # it should have finished: its end record was lost (dry run D2), it is not driving
            if since and el > LOST_FROZEN_S and expect >= LOST_FROZEN_CM:
                moved = math.hypot(since[-1][1] - since[0][1], since[-1][2] - since[0][2])
                if moved < LOST_FROZEN_FRAC * expect:
                    return ("its wheels should have taken it about %.0f cm but the camera's spot moved %.0f cm (the "
                            "camera may be following something else)" % (expect, moved))
        return ""

    def watchdog_loop(self):
        """The camera watchdog, all session, 10 times a second, for whichever robot the camera follows (finds,
        routes, K legs, parking, idle legs alike): a robot whose centre is, or within 0.3 s will be, closer than
        WATCH_EDGE_CM to an open edge or WATCH_ROBOT_CM to a parked robot, while moving towards it, gets `stop`
        (three times, 0.2 s apart: a lost WiFi packet must not matter). Moving away is always allowed, so it can be
        brought back. The firmware's edge guard and front stop are the first line; this is the second."""
        last = {}
        last_lost = {}
        last_still = {}
        seen_border = time.time()
        while not self.watch_quit.is_set():
            time.sleep(0.1)
            try:
                self.link_tick()
                # a robot's own virtual border (firmware 2026-10-25a): it has stopped itself; Nick decides
                if self.listener is not None and not self.hold.is_set():
                    with self.listener.cv:
                        new_b = [(t_, n_, rec_) for t_, n_, rec_ in self.listener.recs["BORDER"] if t_ > seen_border]
                    for t_, n_, rec_ in new_b:
                        seen_border = max(seen_border, t_)
                        if n_ in self.held_out:
                            continue
                        self.watchdog_fired = True
                        self.watch_stop = "virtual border (the robot's own)"
                        if self.forward_explore_leg(n_):
                            # explore driving forwards (its edge guard on) back towards a start near the edge: it has
                            # stopped itself at the border, which is all that was needed; explore goes on from there
                            # (simulator 2026-10-24, kept poses: every return leg held the session)
                            self.say("    %s stopped itself at the virtual border (forwards, exploring): it carries on "
                                     "from there" % n_, quiet=True)
                            continue
                        # (no `border clear` after "carry on": the robot still refuses moves towards the edge and
                        # lets go by itself once back inside; moves away and along stay allowed)
                        self.hold_for_nick(n_, "border", "it stopped itself %s cm from an open edge (its own border "
                                           "check)" % rec_.get("dist_cm", "?"))
                if not self.hold.is_set():
                    # item 4: a tilted robot (its own sensor, any robot streaming RAW) is stopped at once
                    for n_ in list(self.st.get("robots") or []):
                        if n_ in self.held_out or time.time() - (getattr(self, "tilt_said", {}) or {}).get(n_, 0) < 30:
                            continue
                        why_t = self.tilt_why(n_)
                        if why_t:
                            self.__dict__.setdefault("tilt_said", {})[n_] = time.time()
                            self.hung_stop(n_, "tilt", why_t)
                robot = self.tracker.active if self.tracker is not None else None
                if not robot or self.hold.is_set() or robot in self.held_out:
                    continue
                now_s = time.time()
                why_s = self.stuck_why(robot, now_s) if now_s - last_lost.get(robot, 0.0) > 3.0 else ""
                if why_s:
                    last_lost[robot] = now_s
                    self.hung_stop(robot, "stuck", why_s)
                    continue
                # a driving leg the camera has lost (Nick, 2026-10-24: stop and wait): no positions for LOST_DRIVE_S,
                # or positions that do not move while the wheels drive (the tracker stuck on the old spot, as in
                # stage 3: 2026-10-24 review)
                now = time.time()
                why_l = self.lost_why(robot, now) if now - last_lost.get(robot, 0.0) > 3.0 else ""
                if why_l:
                    last_lost[robot] = now
                    self.watchdog_fired = True
                    self.watch_stop = "lost while driving"
                    self.hold_for_nick(robot, "lost", why_l)
                    continue
                self.camera_still_check(robot, last_still)
                smp = self.history.since(time.time() - 0.6)
                if len(smp) < 3 or time.time() - smp[-1][0] > 1.0:
                    continue
                why_b = self.border_check(robot, smp)
                if getattr(self.a, "home", False):
                    self.border_shadow(robot, smp, bool(why_b))
                if not why_b and self.speed_guard_check(robot, smp, last):
                    continue
                if why_b and self.forward_explore_leg(robot):
                    # the same for the camera's border: stop that robot, no hold (backwards never gets here)
                    if time.time() - last.get(robot, 0) > 2.0:
                        last[robot] = time.time()
                        for _ in range(3):
                            self.link.send_name(robot, "stop")
                            time.sleep(0.1)
                        self.say("    %s: stopped at the virtual border (forwards, exploring): %s" % (robot, why_b),
                                 quiet=True)
                    continue
                if why_b:
                    self.watchdog_fired = True
                    self.watch_stop = "virtual border"
                    self.st.setdefault("watchdog_stops", []).append([round(time.time(), 1), robot, "border: " + why_b,
                                                                     self.order])
                    self.hold_for_nick(robot, "border", why_b)
                    continue
                if not self.may_move(robot):
                    # commanded still (no leg, no firmware run, no find, not just after one): what the camera sees
                    # moving is the tracker settling, not the robot (dry run 19:11: at the bottom edge the tracker
                    # settled 5 cm after a find and the watchdog stopped Left's next move before it began)
                    continue
                parked = {n: tuple(q[:2]) for n, q in self.st["positions"].items() if n != robot and q}
                # a slow camera: the last 1.2 s (hazard needs 4 pictures), and the parked robots' clearance widened
                # like the border, by how far this robot drives in the lag above budget (speed guard)
                smp_w = self.history.since(time.time() - 1.2)
                robot_cm = PL.WATCH_ROBOT_CM + self.speed_guard_cm(robot)
                if self.board is None:
                    if not getattr(self, "exploring", False):
                        continue
                    # exploring (no board yet): the parked-robot half only, in the provisional frame that the
                    # positions and the parked spots share (2026-10-25 15:10: this returned before it, and Left's
                    # home leg pushed Center for 7 s); the open edges are the border's job (the guard) meanwhile
                    why = ST.hazard(smp_w, None, parked, robot_cm=robot_cm)
                else:
                    edge_cm = -1e9 if robot == getattr(self, "edge_touch", None) else self.watch_edge_cm
                    if self.backing_out(robot):
                        edge_cm = -1e9       # fix 4: backing straight out after its own edge stop: always allowed
                    jump = None
                    if self.backing_out(robot):
                        pass
                    elif robot in self.seek:
                        # fix 2: the heading crawl (and explore's crawls) look for the edge on purpose; the edge
                        # sensors stop them. The watchdog stays as the last backstop, nearer; and at 2 cm/s anything
                        # much faster is the tracker settling (dry run 19:06: its first lock after a find jumped 5 cm
                        # and stopped the crawl as it was sent)
                        edge_cm = min(edge_cm if edge_cm is not None else PL.WATCH_EDGE_CM, CRAWL_WATCH_EDGE_CM)
                        jump = CRAWL_JUMP_CMS
                    why = ST.hazard(smp_w, self.board, parked, edge_cm=edge_cm, robot_cm=robot_cm, jump_cms=jump)
                if not why and getattr(self, "bricks_guard", None) is not None:      # the brick demo's objects
                    import e02_bricks as BR
                    why = BR.object_hazard(smp, self.bricks_guard)
                if not why or time.time() - last.get(robot, 0) < 2.0:
                    continue
                last[robot] = time.time()
                for _ in range(3):
                    self.send(robot, "stop")
                    time.sleep(0.2)
                self.watchdog_fired = True
                self.watch_stop = why
                self.st.setdefault("watchdog_stops", []).append([round(time.time(), 1), robot, why, self.order])
                self.say("  STOPPED %s (camera watchdog): %s" % (robot, why))
                self.mark("e02 %s stop %s" % ("near-robot" if "parked" in (why or "") else "watchdog", robot))
            except Exception as e:          # the watchdog must never die
                self.say("  (watchdog: %s)" % e, quiet=True)

    # --- the naming check ---------------------------------------------------------------------------
    def naming_check(self):
        """2 minutes, everything parked, all blinking their codes (`run gyro` 120 s, which also measures each
        gyro's drift), the blink tracker of tools/live naming them. Recorded only."""
        names = [n for n in self.st["robots"] if n not in self.unavailable]
        secs = self.a.naming_s
        self.say("=== Naming check: %d s, the robots stand still blinking their codes ===" % secs)
        live = None
        try:
            live = NamingWatch(self.cam, self.dir)
            self.feed.hooks.append(live.push)
        except Exception as e:      # tools/live missing a piece: the check is data only
            self.say("  (the blink tracker could not start: %s; recording the check without it)" % e)
        self.mark("e02 naming check start")
        t0 = time.time()
        for n in names:
            self.send(n, "set gyro_check_s %d" % secs)
        time.sleep(0.3)
        for n in names:
            self.send(n, "run gyro")
        time.sleep(secs + 6)
        self.mark("e02 naming check end")
        if live:
            self.feed.hooks.remove(live.push)
        drift = {}
        for n in names:
            r = self.listener.latest("GYRO", n, t0)
            if r:
                drift[n] = num(r[1].get("raw_drift_dps"))
        known = {n: self.st["positions"].get(n) for n in names}
        result = dict(t0=t0, seconds=secs, gyro_raw_drift_dps=drift,
                      blink=live.summary(known) if live else None)
        self.st["naming_check"] = result
        with open(os.path.join(self.dir, "naming_check.json"), "w") as f:
            json.dump(result, f, indent=1)
        if live:
            for n, r in result["blink"]["robots"].items():
                self.say("  %s: %s" % (n, r["words"]))
        for n, d in drift.items():
            if d is not None:
                self.say("  %s's gyro creeps %+.3f deg/s standing still (starting point for its drift correction)" % (n, d))
        self.st.save()

    # --- corrections ------------------------------------------------------------------------------
    def prior(self, robot):
        p = dict(FIT.PRIOR)
        d = ((self.st.get("naming_check") or {}).get("gyro_raw_drift_dps") or {}).get(robot)
        if d is not None:
            p["gyro_drift_dps"] = round(p["gyro_scale"] * d, 4)
        return p

    def fitted(self, robot):
        return (self.st["fits"].get(robot) or {}).get("values")

    def move_corrections(self, robot):
        """For moves between runs (parking): the latest learned values, else the priors."""
        return self.fitted(robot) or self.prior(robot)

    def set_corrections(self, robot, values):
        """U / K: factory values (1, 0, 1); C / KC / moves: the given ones. Sent as `set` (verified against the
        run's PARAM records); `corr reset` would do the same for U but also rewrite the robot's saved
        corrections, and the simulator does not have `corr`."""
        if values is None:
            values = dict(gyro_scale=1.0, gyro_drift_dps=0.0, wheel_scale=1.0)
        if self.degrade.spec.get("wheels") or self.degrade.spec.get("gyro"):
            values = self.degrade.corrections(robot, values)     # --degrade: a bad wheel or gyro, this run's amount
        cmds = ["set gyro_scale %.4f" % values["gyro_scale"], "set gyro_drift_dps %.4f" % values["gyro_drift_dps"],
                "set wheel_scale %.4f" % values["wheel_scale"]]
        for c in cmds:
            self.send(robot, c)
            time.sleep(0.05)
        return cmds

    def learn(self, robot, run_row, t0, t1, runs):
        """After a U run: pair the camera's straight stretches with the robot's raw counting, refit."""
        cam = [(s[0], s[1], s[2]) for s in self.history.since(t0, t1)]
        stretches = FIT.camera_stretches(cam)
        obs = []
        for run in runs:
            raw = self.listener.raw.get(run) or []
            if self.degrade:
                raw = self.degrade.raw_rows(robot, raw)     # --degrade: what a bad encoder or gyro reports
            info = self.listener.runinfo.get(run) or {}
            ys = num(info.get("yaw_sign")) or 1
            ser = FIT.raw_series(raw, yaw_sign=1 if ys >= 0 else -1)
            if ser is None:
                continue
            k = (ser["t"] >= ser["t"][0] - 0.1) & (ser["t"] <= ser["t"][-1] + 0.1)
            mine = [s for s in stretches if s["t0"] >= ser["t"][k][0] and s["t1"] <= ser["t"][k][-1]]
            obs += FIT.pair(mine, ser, run=run)
        data = self.st["fit_data"].setdefault(robot, [])
        data += obs
        prev = self.fitted(robot)
        measured_drift = ((self.st.get("naming_check") or {}).get("gyro_raw_drift_dps") or {}).get(robot) is not None
        pr = self.prior(robot)
        gate = self.trust.wheel_gate_cm(override=getattr(self.a, "fit_max_rms_cm", None))
        res = FIT.fit_robot(data, prev, prior=pr,
                            prior_sd=dict(gyro_drift_dps=0.1) if measured_drift else None,
                            measured_drift=pr["gyro_drift_dps"] if measured_drift else None,
                            max_rms_cm=gate)
        # the wheels' and gyro's trust: this run's own stretches against the values fitted to ALL runs (a run
        # fitted on its own would absorb a sensor that is off by a different amount every run)
        if obs:
            rc, rd = stretch_rms_against(obs, res.get("fitted") or {}, pr)
            self.trust.stretch_fit(robot, rc, rd, len(obs))
            # and each run's wheel scale on its own (e02_trust runs check: the robot against itself, run to run)
            if hasattr(self.trust, "run_scale"):
                for _run, sc_, len_, pairs_ in run_scales(obs):
                    try:
                        self.trust.run_scale(robot, sc_, len_, pairs=pairs_)
                    except TypeError:                   # an older e02_trust
                        self.trust.run_scale(robot, sc_, len_)
            self.trust_note()
        entry = dict(t=round(time.time(), 1), order=run_row["order"], n_new=len(obs), n_total=len(data), gate_cm=round(gate, 2),
                     n_stretches=len(stretches), accepted=res["accepted"], fitted=res["fitted"], reasons=res["reasons"],
                     values=res["values"], gyro_ok=res["gyro_ok"], wheel_ok=res["wheel_ok"])
        f = self.st["fits"].setdefault(robot, dict(values=None, history=[]))
        f["history"].append(entry)
        if res["values"] and res["accepted"]:
            f["values"] = res["values"]
        run_row["fit_obs"] = len(obs)
        self.write_fits()
        words = FIT.describe(f["values"])
        self.say("  Self-calibration %s (%d stretches from this run, %d in all): %s%s" % (
            robot, len(obs), len(data), words, ("; " + "; ".join(res["reasons"])) if res["reasons"] else ""))

    # --- one run ----------------------------------------------------------------------------------
    def send_back_aside(self):
        """Robots that stepped aside for another (make_way) go back to their own spots, before the next run starts
        (every other robot is parked by then)."""
        while self.return_after:
            m = self.return_after.pop(0)
            if m in self.unavailable or not self.activate(m):
                continue
            ok_, why_ = self.park(m)
            self.say("    %s: back to its spot%s" % (m, "" if ok_ else " failed (%s)" % why_))
        self.active = None

    def recheck_routes(self, why):
        """After the map changes (a board found gone: its side is an open edge now), every route is checked again
        from the start spot (dry run 2026-10-23 23:20: the straight route ran at the edge where the board had been
        and the watchdog stopped it). Routes that no longer fit are left out: their pending runs are put aside,
        said, and kept in the plan's record (status left_out)."""
        out = []
        for rt in PL.ROUTES:
            ok, open_room, board_room = self.board.route_fits(rt)
            if not ok:
                out.append(rt)
                self.say("  Route %s no longer fits (%s): front %.0f cm from an open edge, body %.1f cm from a board"
                         % (PL.ROUTE_NAMES[rt], why, open_room, board_room))
        prev = set(self.st["setup"].get("routes_left_out") or [])
        self.st["setup"]["routes_left_out"] = sorted(prev | set(out))
        n = 0
        for r in self.st["schedule"]:
            if r["status"] == "pending" and r["route"] in out:
                r["status"] = "left_out"
                r.setdefault("notes", []).append("route left out: %s" % why)
                n += 1
        if n:
            self.say("  %d pending run(s) on those routes are put aside (left_out)." % n)
        self.st.save()
        return out

    def route_blocked_by_aside(self, route):
        """Fix 7: (name, cm) of the first set-aside robot (put aside, left out) whose centre is within
        PL.WATCH_ROBOT_CM + ASIDE_ROUTE_MARGIN_CM of the route's path from the start spot, else None."""
        if self.board is None:
            return None
        aside = [n for n in self.unavailable if (self.st.get("positions") or {}).get(n)]
        if not aside:
            return None
        sx, sy, _ = self.board.nominal_start()
        out, back = self.board.route_floor(route, (sx, sy))
        pts = [(sx, sy)] + [tuple(q) for q in out] + [tuple(q) for q in back]
        lim = PL.WATCH_ROBOT_CM + ASIDE_ROUTE_MARGIN_CM
        best = None
        for n in aside:
            q = self.st["positions"][n]
            d = PL.dist_to_path((q[0], q[1]), pts)
            if d < lim and (best is None or d < best[1]):
                best = (n, d)
        return best

    def do_run(self, r):
        robot, cond, route = r["robot"], r["cond"], r["route"]
        st = self.st
        if route in (st["setup"].get("routes_left_out") or []):
            r["status"] = "left_out"
            r.setdefault("notes", []).append("route left out after the map changed")
            st.save()
            return
        blk = self.route_blocked_by_aside(route)
        if blk:
            # fix 7 (15:45: 4 of 5 square runs cut by watchdog stops 14-17 cm from put-aside Right in the square's
            # path): a route that passes a set-aside robot is not run
            r["status"] = "left_out"
            r.setdefault("notes", []).append("route passes set-aside %s (%.0f cm)" % blk)
            self.st.setdefault("routes_blocked", []).append([round(time.time(), 1), r["order"], route, blk[0],
                                                             round(blk[1], 1)])
            self.say("  Run %s (%s, route %s) is not run: the route passes %s, which is set aside there, %.0f cm "
                     "from its path." % (r["order"], robot, PL.ROUTE_NAMES[route], blk[0], blk[1]))
            st.save()
            return
        if self.return_after:
            self.send_back_aside()
        done, total = st.counts()
        el = time.time() - st["t_session_start"] + st.get("elapsed_before", 0)
        left = (total - done) * st.mean_run_s()
        print("")
        if getattr(self, "load", None) is not None and self.load._thread is None:
            self.say("  Planned load from now on: %s, %.0f s on and %.0f s off (load.csv in the state; marked on the "
                     "video)." % (self.load.spec, self.load.on_s, self.load.off_s))
            self.load.start()
        self.say("=== Run %d of %d (block %d): %s, %s (%s), route %s | %s in, about %s left ===" % (
            done + 1, total, r["block"], robot, cond, PL.COND_WORDS[cond], PL.ROUTE_NAMES[route], hms(el), hms(left)))
        if self.calib_note():
            self.say("    (%s)" % self.calib_note())
            r["calibration"] = self.calib_note()
        if self.degrade:
            self.say("    (%s)" % self.degrade.words())
            r["degraded"] = self.degrade.words()
            r.setdefault("notes", []).append(self.degrade.words())
        if self.last_result:
            self.say("    last result: " + self.last_result)
        r["status"] = "running"
        r["t_start"] = round(time.time(), 2)
        st.save()
        self.order = str(r["order"])
        if not self.activate(robot):
            ok = self.recover(robot, "the camera can't see it at its spot before its run")
            if not ok:
                r["status"] = "pending"
                st.save()
                return
        mode = 1 if cond == "K" else 0
        tag = "%d/%d %s %s r%d" % (done + 1, total, robot, cond, route)
        # 1. find: every find with the same corrections (the latest learned, else the priors), so the routes
        # all start from an equally good find; the condition's corrections are set after it (the firmware keeps
        # the heading when they change)
        find_corr = self.move_corrections(robot)
        self.end_moves(robot)
        fcmds = self.set_corrections(robot, find_corr)
        rdeg = self.board.route_heading(route)
        params = self.board_params() + ["set e02_route %d" % route, "set e02_mode %d" % mode,
                                        "set e02_route_deg %.1f" % rdeg]
        for c in params:
            self.send(robot, c)
            time.sleep(0.05)
        t_run0 = time.time()
        fnd = self.do_find(robot, tag)
        r["find_run"], r["find_result"] = fnd["run"], fnd.get("result")
        r["find_board_lines"] = fnd.get("board_lines")
        r["find_corr"] = find_corr
        if fnd["run"]:
            bad, unseen = check_params(self.watch, fnd["run"], list(params) + fcmds)
            if bad:
                self.say("  %s did not get every setting (a WiFi packet was lost): %s. Redoing this run." % (robot, "; ".join(bad)))
                r["redos"] = r.get("redos", 0) + 1
                r["status"] = "pending" if r["redos"] < 3 else "failed"
                r.setdefault("notes", []).append("settings lost: " + "; ".join(bad))
                self.park(robot)
                st.save()
                return
        if not fnd["ok"]:
            self.say("  Find failed: %s" % fnd["words"])
            r.setdefault("notes", []).append("find failed: " + fnd["words"])
            if r.get("find_redos", 0) < 1:
                # a missed bump or a nudged robot: park it and do this run again later in its block (once)
                r["find_redos"] = r.get("find_redos", 0) + 1
                r["status"] = "pending"
                st.save()
                self.say("  This run is done again later in its block.")
                self.set_corrections(robot, self.move_corrections(robot))
                if not self.recover(robot, "find failed"):
                    self.say("  %s could not be parked; its remaining runs are put aside." % robot)
                self.active = None
                return
            r.setdefault("flags", []).append("find_failed")
            self.finish_failed(r, robot, "find failed: " + fnd["words"])
            return
        # 2. the condition's corrections. Mode 0 (U, C, KC): the transit to the start spot and the start turn run on
        # the find's corrections; the firmware switches to the condition's at the route's start (e02_route_*), so
        # the route alone is what is measured. K: factory values, set now (the laptop steers it all the way).
        source = "factory"
        if cond in ("C", "KC"):
            vals = self.fitted(robot)
            if vals is None:
                vals = self.prior(robot)
                source = "prior (no learned values yet)"
                self.say("  No learned corrections for %s yet: this %s run uses the priors (flagged)." % (robot, cond))
                r.setdefault("flags", []).append("prior_corrections")
            else:
                source = "learned"
        else:
            vals = dict(gyro_scale=1.0, gyro_drift_dps=0.0, wheel_scale=1.0)
        r["corr"] = vals
        if mode == 0:
            cmds = ["set e02_route_gyro_scale %.4f" % vals["gyro_scale"],
                    "set e02_route_gyro_drift_dps %.4f" % vals["gyro_drift_dps"],
                    "set e02_route_wheel_scale %.4f" % vals["wheel_scale"],
                    # KC's fixes are weighted on the laptop (kc_fixes): the firmware takes each as sent
                    "set e02_fix_k 1",
                    # the stand-still after every leg, from this session's picture delay (same for U, C and KC)
                    "set e02_leg_pause_ms %d" % self.leg_pause_ms()]
            for c in cmds:
                self.send(robot, c)
                time.sleep(0.05)
            cmds = cmds + fcmds
        else:
            cmds = self.set_corrections(robot, vals) + ["set e02_route_gyro_scale -999",
                                                         "set e02_route_gyro_drift_dps -999",
                                                         "set e02_route_wheel_scale -999"]
            for c in cmds[3:]:
                self.send(robot, c)
                time.sleep(0.05)
        self.fw_border([robot])
        r["corr_source"] = source
        time.sleep(0.3)
        # 3. the route
        self.phase = "route"
        pre = self.settle(min_s=0.5)
        nominal = rdeg
        t_e0 = time.time()
        self.mark("e02 %s route start" % tag)
        run = self.start_run(robot, "run e02")
        if not run:
            r.setdefault("flags", []).append("e02_no_start")
            self.finish_failed(r, robot, "did not start `run e02`")
            return
        r["e02_run"] = run
        bad, _ = check_params(self.watch, run, cmds + ["set e02_route %d" % route, "set e02_mode %d" % mode])
        if bad:
            self.send(robot, "stop")
            self.watch.wait_end(run, 5)
            self.say("  %s did not get every setting (a WiFi packet was lost): %s. Redoing this run." % (robot, "; ".join(bad)))
            r["redos"] = r.get("redos", 0) + 1
            r["status"] = "pending" if r["redos"] < 3 else "failed"
            r.setdefault("notes", []).append("settings lost: " + "; ".join(bad))
            self.park(robot)
            st.save()
            return
        prm = self.watch.params.get(run, {})
        k_info = None
        self.watchdog_fired = False
        self.watch_stop = None
        if cond == "KC":
            # where the robot believes it is at each stop: its route's waypoints (it drives to each until its own
            # pose says it is there, and again at the far point and the end if more than e02_redo_cm off). 2026-10-24
            # dry run: the RAW rows' x/y are its raw odometry, not this pose (a fix 138 cm off sent it away).
            sx_, sy_, _ = self.board.nominal_start()
            out_w, back_w = self.board.route_floor(route, (sx_, sy_), rdeg)
            r["kc_waypoints"] = [[round(sx_, 2), round(sy_, 2)]] + [[round(a, 2), round(b, 2)] for a, b in out_w + back_w]
            self.kc_stop.clear()
            th = threading.Thread(target=self.kc_fixes, args=(robot, r, run), daemon=True)
            th.start()
        if cond == "K":
            k_info = self.k_drive(robot, run, route, prm, nominal)
            end = self.watch.wait_end(run, 15) or dict(result="no end record")
        else:
            end = self.watch.wait_end(run, E02_TIMEOUT_S[0], poll=lambda: self.watchdog(robot))
            if end is None:
                self.send(robot, "stop")
                end = self.watch.wait_end(run, 5) or dict(result="no end record")
        self.kc_stop.set()
        self.moves.pop(robot, None)
        e02_end = self.listener.latest("E02", robot, t_e0, run=run)
        eth = num(e02_end[1].get("end_th")) if e02_end else None
        # where it faces now: its own belief at the end (a stopped route may face anywhere); the next leg
        # measures it again
        self.heads[robot] = [eth if eth is not None else nominal, False]
        t_e1 = time.time()
        time.sleep(0.5)
        endp = self.settle(t_from=time.time() - 0.8, min_s=0.5)
        if endp is not None and self.history_raw is not None:
            # a degraded camera (--degrade): the run is scored by the true camera, not the one the runner steers by
            raw_ = self.history_raw.since(time.time() - 0.8)
            if raw_:
                endp = (float(np.median([q[1] for q in raw_])), float(np.median([q[2] for q in raw_])))
        self.mark("e02 %s route end %s" % (tag, end["result"]))
        r["e02_result"] = end["result"]
        r["battery"] = end.get("battery")
        # 4. what the camera measured: start, far point and end, at the moments the robot logged them
        e02 = self.listener.latest("E02", robot, t_e0, run=run)
        rec = e02[1] if e02 else {}
        raw = self.listener.raw.get(run) or []
        off = FIT.clock_offset(raw)

        def lap(ms_key):
            ms = num(rec.get(ms_key))
            return None if ms is None or off is None else ms / 1000.0 + off
        r["t_find_l"], r["t_e02_l"], r["t_end_l"] = round(t_run0, 3), round(t_e0, 3), round(t_e1, 3)
        for k in ("ms_start", "ms_far", "ms_end"):
            if lap(k) is not None:
                r["t_%s_l" % k[3:]] = round(lap(k), 3)
        if k_info:
            for k in ("t_start", "t_far"):
                if k_info.get(k):
                    r[k + "_l"] = round(k_info[k], 3)

        def cam_at(ms_key, span):
            t = lap(ms_key)
            if t is None:
                return None
            smp = self.score_hist().since(t + 0.05, t + 0.05 + span)
            if not smp:
                return None
            return (float(np.median([q[1] for q in smp])), float(np.median([q[2] for q in smp])))
        start = (k_info or {}).get("start") or cam_at("ms_start", 0.4) or pre
        far = (k_info or {}).get("far") or cam_at("ms_far", 0.8 * PAUSE_MS / 1000.0)
        if start is None:
            # 2026-10-23 09:44: a K drive the camera lost from the start (no start sample, no pre-run position): the
            # runner crashed here. Nothing to measure: a failed run (kept in the record)
            r.setdefault("flags", []).append("no_camera_start")
            self.finish_failed(r, robot, "the camera did not see where it started (%s)" % end.get("result"))
            return
        bs = [num(rec.get(k)) for k in ("start_x", "start_y", "far_goal_x", "far_goal_y")]
        k_ended = False
        if k_info and k_info.get("far_goal"):
            goal = k_info["far_goal"]
        elif cond == "K":
            # a K drive that ended before its route was planned: the firmware's own route numbers mean nothing in
            # mode 1 (2026-10-24 Pi plan: Right's K run "intended" a far point at (210, 104), beyond the board)
            out_w, _ = self.board.route_floor(route, start, nominal)
            goal = out_w[-1]
            r.setdefault("flags", []).append("k_drive_ended_early")
            r.setdefault("notes", []).append("far point not scored: the K drive ended before its far point")
            far = None
            k_ended = True
        elif None not in bs:
            goal = (start[0] + bs[2] - bs[0], start[1] + bs[3] - bs[1])     # the robot's own route, from the camera start
        else:
            out_w, _ = self.board.route_floor(route, start, nominal)
            goal = out_w[-1]
            r.setdefault("flags", []).append("goal_from_nominal_route")
        if far is None and not k_ended:
            far = self.longest_still(t_e0, t_e1, goal)
            if far:
                r.setdefault("flags", []).append("far_from_stillness")
        if goal is not None and self.board is not None and \
                not (self.board.x0 - 1.0 <= goal[0] <= self.board.x1 + 1.0 and self.board.y0 - 1.0 <= goal[1] <= self.board.y1 + 1.0):
            r.setdefault("flags", []).append("intended_far_off_board")
            r.setdefault("notes", []).append("far point not scored: the intended far point (%.0f, %.0f) is off the "
                                             "board" % (goal[0], goal[1]))
            far = None
        # 2026-10-24 Pi plan: three K/KC runs lost the robot on the camera and their "far point" was only where the
        # camera last saw it (50-127 cm "errors"). The far point counts only when the camera saw the robot around
        # the far time and did not lose or re-find it between the start and then.
        t_far = (k_info or {}).get("t_far") or lap("ms_far")
        if far is not None and t_far is not None:
            seen = self.score_hist().since(t_far - 0.5, t_far + 1.5)
            lost = [d_ for d_ in (self.st.get("disagreements") or [])
                    if d_[1] == robot and t_e0 - 1.0 <= d_[0] <= t_far + 1.0]
            if not seen or lost:
                r.setdefault("flags", []).append("far_unseen")
                r.setdefault("notes", []).append("far point not scored: the camera %s" % (
                    "lost it before the far point" if lost else "did not see it at the far point"))
                far = None
        r["cam_start"] = [round(start[0], 2), round(start[1], 2)] if start else None
        r["intended_far"] = [round(goal[0], 2), round(goal[1], 2)]
        r["cam_far"] = [round(far[0], 2), round(far[1], 2)] if far else None
        r["cam_end"] = [round(endp[0], 2), round(endp[1], 2)] if endp else None
        self.measured_frames(r, [("start", lap("ms_start"), start, None), ("far", lap("ms_far"), far, goal),
                                 ("end", t_e1, endp, start)])
        far_err = math.hypot(far[0] - goal[0], far[1] - goal[1]) if far else None
        end_err = math.hypot(endp[0] - start[0], endp[1] - start[1]) if (endp and start) else None
        r["far_err_cm"] = round(far_err, 2) if far_err is not None else None
        r["end_err_cm"] = round(end_err, 2) if end_err is not None else None
        # the same, with the camera corrected by its error at each place (tape marks, finds: e02_camerr). The far
        # point is where KC should beat C (2026-10-24 review): scored here against the tape-checked camera.
        if any(sm.get("source") == "mark" for sm in self.camerr.samples) and start and far:
            cs, cf = self.camerr.corrected(*start)[:2], self.camerr.corrected(*far)[:2]
            gx, gy = goal[0] - start[0] + cs[0], goal[1] - start[1] + cs[1]
            r["far_err_tape_cm"] = round(math.hypot(cf[0] - gx, cf[1] - gy), 2)
            if endp:
                ce = self.camerr.corrected(*endp)[:2]
                r["end_err_tape_cm"] = round(math.hypot(ce[0] - cs[0], ce[1] - cs[1]), 2)
        bel = None
        sx, sy, ex, ey = (num(rec.get(k)) for k in ("start_x", "start_y", "end_x", "end_y"))
        if None not in (sx, sy, ex, ey):
            bel = math.hypot(ex - sx, ey - sy)
        r["fixes_used"] = num(rec.get("fixes_used"))
        r["robot_thought_end_cm"] = round(bel, 2) if bel is not None else None
        if k_info:
            r["legs"] = k_info["legs"]
        on_board = endp is not None and self.board.on_board(endp[0], endp[1], margin=-1.0)
        verdict, why = ST.judge_run(far_err, end_err, on_board=on_board, seen=endp is not None,
                                    recover_cm=self.a.recover_cm,
                                    runend="watchdog stop" if self.watchdog_fired else (
                                        "runner stop (finding it again)" if (robot in self.runner_stopped and
                                                                             "stop command" in str(end["result"]))
                                        else end["result"]))
        self.runner_stopped.discard(robot)
        r["recovery"] = verdict
        r["verdict"] = verdict or "ok"
        if end["result"] != "done":
            r.setdefault("flags", []).append("runend:" + end["result"])
        self.last_result = "%s %s %s: far point %s, back at start %s%s" % (
            robot, cond, PL.ROUTE_SHORT[route], "%.1f cm off" % far_err if far_err is not None else "not seen",
            "%.1f cm off" % end_err if end_err is not None else "not seen",
            " (robot thought %.1f cm)" % bel if bel is not None else "")
        self.say("  Result: " + self.last_result)
        if verdict:
            r.setdefault("flags", []).append("recovery:" + verdict)
            self.say("  Needed recovery: %s (noted; kept in the data, not an outlier)." % why)
            self.mark("e02 %s recovery %s" % (tag, verdict))
        # 4. learn (U runs)
        if cond == "U":           # any U run, finished or stopped: the raw gyro and wheel counts are still true
            try:
                self.learn(robot, r, t_run0, t_e1 + 0.5, [x for x in (fnd["run"], run) if x])
            except Exception as e:      # a fit problem must never stop the session
                self.say("  (self-calibration skipped this run: %s)" % e)
        # 5. park (this also drives a lost robot back)
        for c in ("set e02_route_gyro_scale -999", "set e02_route_gyro_drift_dps -999", "set e02_route_wheel_scale -999"):
            self.send(robot, c)
        self.set_corrections(robot, self.move_corrections(robot))
        r["status"] = "done"
        r["t_end"] = round(time.time(), 2)
        st.d.setdefault("durations", []).append(round(r["t_end"] - r["t_start"], 1))
        bat = num(r.get("battery"))
        if bat is not None and bat < BATTERY_STOP:
            self.say("  %s's battery is at %d%%: its remaining runs are put aside (charge it, then --resume)." % (robot, bat))
            self.unavailable.add(robot)
        st.save()
        self.write_runs()
        if not self.recover(robot, why if verdict else None):
            self.say("  %s could not be parked; its remaining runs are put aside." % robot)
        self.active = None

    def score_hist(self):
        """The camera samples runs are scored by: the undegraded ones in a --degrade camera test (2026-10-24 review:
        a degraded camera must not also be the ruler), and never cleared by a re-find."""
        return self.history_raw if self.history_raw is not None else self.history

    def longest_still(self, t0, t1, near):
        """The longest still period between t0 and t1 (ties: nearest to `near`): fallback far point."""
        smp = self.score_hist().since(t0, t1)
        best = None
        i = 0
        while i < len(smp):
            j = i
            while j + 1 < len(smp) and math.hypot(smp[j + 1][1] - smp[i][1], smp[j + 1][2] - smp[i][2]) < 0.6:
                j += 1
            dur = smp[j][0] - smp[i][0]
            if dur >= 0.6:
                x = float(np.median([s[1] for s in smp[i:j + 1]]))
                y = float(np.median([s[2] for s in smp[i:j + 1]]))
                key = (round(dur, 1), -math.hypot(x - near[0], y - near[1]))
                if best is None or key > best[0]:
                    best = (key, (x, y))
            i = j + 1
        return best[1] if best else None

    def recover(self, robot, why=None):
        """Park the robot by camera (after a lost run: 'driven back'). Nick only if that fails."""
        if why:
            self.say("  Driving %s back by camera (%s) ..." % (robot, why))
        ok, words = self.park(robot)
        if ok:
            return True
        self.say("  Parking %s failed: %s" % (robot, words))
        p = self.pos(robot)
        if p is None:
            # look for it by a turn on its spot, near where it was last seen
            self.say("  Looking for %s by a short turn on its spot ..." % robot)
            last = self.tracker.where() or (None, *(self.st["positions"].get(robot) or [None, None])[:2])
            near = (last[1], last[2], 60.0) if last and last[1] is not None else None
            q = self.acquire(robot, step_cm=8.0, near=near)
            if q:
                ok, words = self.park(robot)
                if ok:
                    return True
        return self.help_from_nick(robot, words)

    def finish_failed(self, r, robot, words):
        r["status"] = "failed"
        r.setdefault("notes", []).append(words)
        r["t_end"] = round(time.time(), 2)
        self.last_result = "%s %s %s: %s" % (robot, r["cond"], PL.ROUTE_SHORT[r["route"]], words)
        self.st.save()
        self.write_runs()
        self.send(robot, "stop")
        time.sleep(0.5)
        self.set_corrections(robot, self.move_corrections(robot))
        if not self.recover(robot, words):
            self.say("  %s could not be parked; its remaining runs are put aside." % robot)

    # --- K and KC -----------------------------------------------------------------------------------
    def route_points(self, route, prm, start, heading):
        """The firmware's own route (fw_e02 / e02.h e_waypoints) from its PARAM values: (waypoints on the floor
        from `start` along `heading`, index of the far point)."""
        if route == 1:
            L = prm.get("e02_straight_cm") or PL.STRAIGHT_CM
            w, far = [(L, 0.0), (0.0, 0.0)], 0
        elif route == 2:
            a = prm.get("e02_l_cm") or PL.L_CM
            t = 1.0 if (prm.get("e02_l_turn") if prm.get("e02_l_turn") is not None else 1) >= 0 else -1.0
            w, far = [(a, 0.0), (a, t * a), (a, 0.0), (0.0, 0.0)], 1
        else:
            q = prm.get("e02_square_cm") or PL.SQUARE_CM
            w, far = [(q, 0.0), (q, q), (0.0, q), (0.0, 0.0)], 1
        return PL.rot(w, start[0], start[1], heading), far

    def k_drive(self, robot, run, route, prm, heading):
        """Condition K: the laptop steers it by camera, leg by leg (the robot's own pose is not used): to the
        route's start spot (transit), `mark start`, out to the far point, `mark far` (it stands still there),
        back along the route to the start, turned to the start heading, `mark end`. The same route and start
        spot as the robot drives itself in U / C / KC."""
        info = dict(legs=0, far=None, start=None, far_goal=None)
        self.wait_mode1(robot, time.time() - 10.0)
        self.moves[robot] = dict(run=run, why="K")
        self.heads[robot] = [heading if self.heads.get(robot, [None])[0] is None else self.heads[robot][0], False]
        sx, sy, _ = self.board.nominal_start()
        ok, why = self.drive_to(robot, (sx, sy), final_heading=heading)
        if not ok:
            self.say("  K drive: %s" % why)
            self.mark_leg(robot, "end")
            return info
        start = self.settle(min_s=0.5)
        info["start"] = start
        info["t_start"] = time.time()
        self.mark_leg(robot, "start")
        pts, far_i = self.route_points(route, prm, start or (sx, sy), heading)
        # never a route off the board (2026-10-24 Pi plan): from where it really stands, every waypoint must keep
        # its centre BORDER_CM inside the open edges (the virtual border would hold the session there anyway)
        off = [w for w in pts if self.board.edge_room(w[0], w[1])[0] < self.border_cm()]
        if off:
            self.say("  K drive: from where it stands (%.0f, %.0f) the route would come within %.0f cm of an open edge: "
                     "not driven" % (start[0] if start else sx, start[1] if start else sy,
                                     min(self.board.edge_room(w[0], w[1])[0] for w in pts)))
            self.mark_leg(robot, "end")
            return info
        info["far_goal"] = pts[far_i]
        for k, w in enumerate(pts):
            last = k == len(pts) - 1
            ok, why = self.drive_to(robot, w, final_heading=heading if last else None, run=run,
                                    arrive_cm=ST.ARRIVE_CM if (last or k == far_i) else 2.0)
            if not ok:
                self.say("  K drive: %s" % why)
                break
            if k == far_i:
                t = time.time()
                info["t_far"] = t
                self.mark_leg(robot, "far")                 # it stands still e02_pause_ms, then answers
                smp = self.history.since(t, time.time())
                if smp:
                    info["far"] = (float(np.median([q[1] for q in smp])), float(np.median([q[2] for q in smp])))
        self.mark_leg(robot, "end")
        self.moves.pop(robot, None)
        with self.listener.cv:
            info["legs"] = sum(1 for t, n, rec in self.listener.recs["LEG"]
                               if n == robot and rec.get("run") == run and rec.get("state") == "done")
        return info

    def kc_fixes(self, robot, r, run):
        """Condition KC: ONE camera fix at each stop of its route (2026-10-24 review). Before, every still picture
        that arrived was blended half way, so a stop's correction was 50-94% by chance of timing, and two thirds of
        the fixes arrived after it had moved on. Now, at each stop (kc_fix_from: still pictures after it stopped):
          - the camera's position, corrected by the camera's error at that place (e02_camerr), and how sure that is;
          - the robot's own pose there (its RAW rows) and how sure that is (the uncertainty model, from what it
            drove and turned since its last fix);
          - the two weighted by how sure each is (CE.blend_weight), sent as the new pose; the firmware replaces
            its pose with it (e02_fix_k 1 for KC: the weighting is done here).
        The heading goes with a fix only when the camera saw a straight stretch end just before and the gyro says
        it has not turned since, weighted the same way (KC_HEAD_SD_DEG against the gyro's own). Every stop is
        logged (kc_stops.csv)."""
        n = nh = 0
        period = 1.0 / self.a.fix_hz
        done = []                                  # t_stop of the stops already fixed
        last = dict(i=0, dist=0.0, turned=0.0)     # the waypoint of the last fix (0: the start, from the find)
        wps = r.get("kc_waypoints") or []
        fixed_wps = set()                          # 2026-10-24 dry run: 12 fixes at one far point (jitter moved
        # t_stop, so each looked like a new stop) compounded towards the camera as the half-way blend had
        while not self.kc_stop.wait(period):
            recent = self.history.since(time.time() - 3.0)
            fx = kc_fix_from(recent)
            if fx is None:
                continue                          # moving, or not still long enough: no picture of where it is now
            x, y, t_pic, t_stop, seg = fx
            if any(abs(t_stop - d) < 0.5 for d in done):
                continue                          # this stop has had its fix
            own = kc_belief(wps, (x, y), last["i"])
            if own is None:
                continue                          # the camera is not near any waypoint still ahead: not sure where
            i, (bx, by), dist, turned = own
            if i in fixed_wps:
                continue                          # one fix per waypoint: the camera's jitter is not a new stop
            bth = None
            cx, cy, csd = self.camerr.corrected(x, y)
            if self.quality.poor(x, y):
                csd *= KC_POOR_CELL_SD_FACTOR        # the camera sees robots badly here (glare): trust it less
            up_ = self.unc_params(robot)
            rsd = float(UNC.sd_of(up_, dist, turned))
            if not self.trust.bad("camera"):
                # a KC stop is a correction too (its belief against the camera, corrected): the model learns from it
                self.unc_learn(robot, (bx, by), (x, y), kind="kc", dist=dist, turned=turned)
            # what has been learned about each sensor (e02_trust): a robot whose wheels look noisy is less sure of
            # itself, a camera that looks noisy counts for less; and a belief the camera contradicts far beyond
            # both is wrong (2026-10-24: Right's KC run started 12 cm from where it believed)
            rsd, csd = self.trust.kc_sds(robot, rsd, csd)
            w, rejected = TR.kc_gate((bx, by), (cx, cy), rsd, csd)
            if rejected and getattr(self.tracker, "source", None) not in ("both", "lock"):
                # the camera's own position is not firm (detector only, or held): a jump must not rewrite the
                # robot's pose (2026-10-24 review): the usual weighting
                w, rejected = CE.blend_weight(rsd, csd), False
            if rejected:
                r["kc_beliefs_rejected"] = r.get("kc_beliefs_rejected", 0) + 1
                self.say("    %s: KC: the camera puts it %.0f cm from where it believes (more than %.0f x both sds): "
                         "its belief is taken as wrong (%.0f%% camera)" % (
                             robot, math.hypot(cx - bx, cy - by), TR.KC_GATE_SIGMA, 100 * w), quiet=True)
            fxp, fyp = bx + w * (cx - bx), by + w * (cy - by)
            th = "-"
            h = ST.heading_from_track(seg, min_cm=3.0, max_rms_cm=0.4) if seg else None
            tsd = UNC.heading_sd(up_, turned)
            wh = None
            if h is not None and bth is not None:
                trn = self.gyro_turn_since(run, t_stop)
                if trn is not None and abs(trn) < 2.0:
                    hc = ST.wrap180(h + trn)
                    wh = CE.blend_weight(tsd, KC_HEAD_SD_DEG)
                    th = "%.1f" % ST.wrap180(bth + wh * ST.wrap180(hc - bth))
                    nh += 1
            self.send(robot, "fix %.2f %.2f %s %.3f" % (fxp, fyp, th, t_pic))
            n += 1
            done.append(t_stop)
            fixed_wps.add(i)
            last.update(i=i)
            row = [round(time.time(), 2), robot, run, r.get("order", ""), round(t_stop, 2), round(bx, 2), round(by, 2),
                   round(x, 2), round(y, 2), round(cx, 2), round(cy, 2), round(csd, 2), round(dist, 1),
                   round(turned, 1), round(rsd, 2), round(w, 3), "" if wh is None else round(wh, 3),
                   round(fxp, 2), round(fyp, 2), th, round(math.hypot(cx - bx, cy - by), 2)]
            self.kc_log(row)
            last.update(dist=0.0, turned=0.0)
        r["fixes_sent"] = n
        r["fixes_with_heading"] = nh

    KC_COLS = ["t", "robot", "run", "order", "t_stop", "own_x", "own_y", "cam_x", "cam_y", "cam_corr_x", "cam_corr_y",
               "cam_sd_cm", "dist_since_fix_cm", "turned_since_fix_deg", "own_sd_cm", "weight", "head_weight",
               "fix_x", "fix_y", "fix_th", "camera_minus_own_cm"]

    def kc_log(self, row):
        path = os.path.join(self.dir, "kc_stops.csv")
        new = not os.path.exists(path)
        with open(path, "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(self.KC_COLS)
            w.writerow(row)

    def gyro_turn_since(self, run, t):
        """How far the robot turned (deg, left +, raw gyro) since laptop time t, from its RAW rows, or None."""
        raw = self.listener.raw.get(run) or []
        if len(raw) < 3:
            return None
        off = FIT.clock_offset(raw[-400:])
        info = self.listener.runinfo.get(run) or {}
        sign = -1.0 if (num(info.get("yaw_sign")) or 1) < 0 else 1.0
        rows = [q for q in raw[-400:] if q["ms"] is not None and q["yaw_deg"] is not None]
        before = [q for q in rows if q["ms"] / 1000.0 + off <= t]
        if not before or not rows:
            return None
        d = ST.wrap180(rows[-1]["yaw_deg"] - before[-1]["yaw_deg"])
        return sign * d

    # --- the dance (Nick, 2026-10-22; e02_dance.py) ------------------------------------------------------
    def dance(self):
        """The 3-robot dance set to a beat: start poses, a count-in, every move on its beat with camera fixes
        between moves and a supervisor that stops all robots if two get too close or one is lost; then parking.
        Returns the result dict (dance_result.json), or None."""
        import e02_dance as DN
        return DN.DanceRun(self, bpm=getattr(self.a, "dance_bpm", 120.0) or 120.0).run()

    # --- files ------------------------------------------------------------------------------------
    RUN_COLS = ["order", "block", "robot", "cond", "route", "status", "recovery", "flags", "find_run", "e02_run",
                "find_result", "e02_result", "t_start", "t_end", "cam_start_x", "cam_start_y", "cam_far_x", "cam_far_y",
                "cam_end_x", "cam_end_y", "intended_far_x", "intended_far_y", "far_err_cm", "end_err_cm",
                "far_err_tape_cm", "end_err_tape_cm",
                "robot_thought_end_cm", "corr_source", "gyro_scale", "gyro_drift_dps", "wheel_scale", "battery",
                "fixes_sent", "fixes_used", "legs", "fit_obs", "notes"]

    def write_runs(self):
        rows = []
        for r in self.st["schedule"]:
            c = r.get("corr") or {}

            def xy(k, i):
                v = r.get(k)
                return v[i] if v else ""
            rows.append(dict(order=r["order"], block=r["block"], robot=r["robot"], cond=r["cond"], route=r["route"],
                             status=r["status"], recovery=r.get("recovery", ""), flags=" ".join(r.get("flags", [])),
                             find_run=r.get("find_run") or "", e02_run=r.get("e02_run") or "",
                             find_result=r.get("find_result") or "", e02_result=r.get("e02_result") or "",
                             t_start=r.get("t_start", ""), t_end=r.get("t_end", ""),
                             cam_start_x=xy("cam_start", 0), cam_start_y=xy("cam_start", 1),
                             cam_far_x=xy("cam_far", 0), cam_far_y=xy("cam_far", 1),
                             cam_end_x=xy("cam_end", 0), cam_end_y=xy("cam_end", 1),
                             intended_far_x=xy("intended_far", 0), intended_far_y=xy("intended_far", 1),
                             far_err_cm=r.get("far_err_cm", ""), end_err_cm=r.get("end_err_cm", ""),
                             far_err_tape_cm=r.get("far_err_tape_cm", ""), end_err_tape_cm=r.get("end_err_tape_cm", ""),
                             robot_thought_end_cm=r.get("robot_thought_end_cm", ""), corr_source=r.get("corr_source", ""),
                             gyro_scale=c.get("gyro_scale", ""), gyro_drift_dps=c.get("gyro_drift_dps", ""),
                             wheel_scale=c.get("wheel_scale", ""), battery=r.get("battery", ""),
                             fixes_sent=r.get("fixes_sent", ""), fixes_used=r.get("fixes_used", ""),
                             legs=r.get("legs", ""), fit_obs=r.get("fit_obs", ""), notes=" | ".join(r.get("notes", []))))
        write_csv(os.path.join(self.dir, "runs.csv"), self.RUN_COLS, rows)
        try:
            self.write_analysis_runs()
        except Exception as e:      # never stop a session over a side file
            self.say("  (e02_runs.csv not written: %s)" % e)

    def write_analysis_runs(self):
        """e02_runs.csv for tools/experiment/e02_analysis.py (lane A's --runs): its column names, times on the
        camera's (Pi's) clock: Pi time = laptop time - (laptop - Pi clock offset) + capture delay."""
        off = self.feed.clock.offset if self.feed is not None and self.feed.clock.offset is not None else None
        if off is not None:
            self.st["setup"]["pi_offset_s"] = round(off - self.feed.cap, 4)
        po = self.st["setup"].get("pi_offset_s")

        def pi(t):
            return "" if (t is None or po is None) else "%.3f" % (t - po)
        rows = []
        for r in self.st["schedule"]:
            if r["status"] not in ("done", "failed"):
                continue
            rows.append(dict(run_id=r.get("e02_run") or "", robot=r["robot"], condition=r["cond"], route=r["route"],
                             block=r["block"], robot_run=r.get("e02_run") or "", find_run=r.get("find_run") or "",
                             order=r["order"], recovery=r.get("recovery") or "",
                             t_find=pi(r.get("t_find_l")), t_start=pi(r.get("t_start_l") or r.get("t_e02_l")),
                             t_far=pi(r.get("t_far_l")), t_end=pi(r.get("t_end_l")), status=r["status"],
                             flags=" ".join(r.get("flags", []))))
        write_csv(os.path.join(self.dir, "e02_runs.csv"), ["run_id", "robot", "condition", "route", "block", "robot_run",
                                                          "find_run", "order", "recovery", "t_find", "t_start", "t_far",
                                                          "t_end", "status", "flags"], rows)

    def write_fits(self):
        cols = ["robot", "t", "order", "accepted", "gyro_ok", "wheel_ok", "n_new", "n_total", "fit_gyro_scale",
                "fit_gyro_drift_dps", "fit_wheel_scale", "se_gyro_scale", "se_gyro_drift_dps", "se_wheel_scale",
                "rms_deg", "rms_cm", "turns", "used_gyro_scale", "used_gyro_drift_dps", "used_wheel_scale", "reasons"]
        rows = []
        for robot, f in self.st["fits"].items():
            for h in f.get("history", []):
                fi, v = h.get("fitted") or {}, h.get("values") or {}
                rows.append(dict(robot=robot, t=h["t"], order=h["order"], accepted=h["accepted"], gyro_ok=h["gyro_ok"],
                                 wheel_ok=h["wheel_ok"], n_new=h["n_new"], n_total=h["n_total"],
                                 fit_gyro_scale=_r(fi.get("gyro_scale")), fit_gyro_drift_dps=_r(fi.get("gyro_drift_dps")),
                                 fit_wheel_scale=_r(fi.get("wheel_scale")), se_gyro_scale=_r(fi.get("se_gyro_scale")),
                                 se_gyro_drift_dps=_r(fi.get("se_gyro_drift_dps")),
                                 se_wheel_scale=_r(fi.get("se_wheel_scale")), rms_deg=_r(fi.get("rms_deg")),
                                 rms_cm=_r(fi.get("rms_cm")), turns=fi.get("turns", ""),
                                 used_gyro_scale=v.get("gyro_scale", ""), used_gyro_drift_dps=v.get("gyro_drift_dps", ""),
                                 used_wheel_scale=v.get("wheel_scale", ""), reasons=" | ".join(h.get("reasons", []))))
        write_csv(os.path.join(self.dir, "fits.csv"), cols, rows)

    # --- the tape-mark check (2026-10-24 review: truth that is not the camera) ---------------------------------
    MARK_NEAR_CM = 15.0     # chosen: a robot put by hand on a mark is within this of where the camera should see it

    def mark_check(self):
        """Nick puts one robot by hand on each tape mark (e02_camerr.MARKS: measured from the boards' faces); the
        camera's view of it there is a sample of the camera's own error (e02_camerr). Optional (--marks); a mark
        can be skipped. The robot goes back to where it was parked afterwards (Nick puts it back)."""
        robots = [n for n in self.st["robots"] if n not in self.unavailable]
        if not robots or self.tracker is None or self.board is None:
            return
        robot = robots[0]
        home = self.st["positions"].get(robot)
        self.say("=== Tape-mark check (the camera against the tape marks) ===")
        self.say("  Lift %s by hand for this; it does not drive. Put its middle over each cross, facing any way." % robot)
        others = [tuple(q[:2]) for n, q in self.st["positions"].items() if n != robot and q]
        got = []
        self.tracker.idle()
        bd = self.board
        mid = ((bd.x0 + bd.x1) / 2.0, (bd.y0 + bd.y1) / 2.0)
        yw = "%s board" % CAM.picture_side(self.cam, bd.yboard_cm, mid[1])      # the board along x = yboard_cm
        xw = "%s board" % CAM.picture_side(self.cam, mid[0], bd.xboard_cm)
        for name, fy, fx in CE.MARKS:
            x, y = CE.mark_xy(self.board, fy, fx)
            ans = self.ask("  Mark %s (%.0f cm from the %s, %.0f cm from the %s): put %s on it, "
                           "take your hand away, press Enter (s = skip this mark, q = stop the check) "
                           % (name, fy, yw, fx, xw, robot), "").lower()
            if ans.startswith("q"):
                break
            if ans.startswith("s"):
                continue
            if self.a.dry_run:
                # the simulator's hand: put it down near the mark (a hand is about 0.5 cm off)
                with open(os.path.join(self.dir, "sim", "PLACE_%s.json" % robot), "w") as fh:
                    json.dump(dict(x=x + 0.4, y=y - 0.3), fh)
                time.sleep(1.0)
            time.sleep(1.0)                                  # still pictures of it standing there
            with self.tracker.lock:
                allc = self.tracker.candidates()            # the detector on the newest picture (it is idle now)
            cands = [c for c in allc if c.get("shape")
                     and math.hypot(c["x"] - x, c["y"] - y) <= self.MARK_NEAR_CM
                     and not any(math.hypot(c["x"] - o[0], c["y"] - o[1]) < CAM.MotionTracker.OTHER_CM for o in others)]
            if not cands:
                self.say("    The camera does not see a robot within %.0f cm of mark %s: skipped." % (self.MARK_NEAR_CM, name))
                continue
            c = min(cands, key=lambda c: math.hypot(c["x"] - x, c["y"] - y))
            self.tracker.follow(robot, c["x"], c["y"])
            self.history.clear()
            p = self.settle(min_s=0.5, timeout=3.0) or (c["x"], c["y"])
            smp = self.camerr_add(x, y, p, "mark", robot)
            got.append(smp)
            self.say("    Mark %s: the camera sees it %.1f cm off (%+.1f, %+.1f)" % (name, math.hypot(smp["dx"], smp["dy"]),
                                                                              smp["dx"], smp["dy"]))
        if home:
            self.ask("  Put %s back on its parking spot (%.0f, %.0f) and press Enter " % (robot, home[0], home[1]), "")
            if self.a.dry_run:
                with open(os.path.join(self.dir, "sim", "PLACE_%s.json" % robot), "w") as fh:
                    json.dump(dict(x=home[0], y=home[1]), fh)
                time.sleep(1.0)
            time.sleep(1.0)
            self.tracker.follow(robot, home[0], home[1])
            self.history.clear()
        self.st["setup"]["marks_done"] = dict(t=round(time.time(), 1), n=len(got), robot=robot)
        if got:
            d = sorted(math.hypot(g["dx"], g["dy"]) for g in got)
            self.say("  Tape marks: the camera is %.1f cm off at the median, %.1f at most (%d marks); KC's fixes are "
                     "corrected by this from now on." % (d[len(d) // 2], d[-1], len(d)))
        self.st.save()

    # --- the session --------------------------------------------------------------------------------
    def start_shift_watch(self):
        """The camera-drift watch (CAM.CameraShiftWatch; 2026-10-24 analysis: the picture crept up to 2 px during
        sessions). From now on positions are corrected for the camera's creep; every measurement is kept
        (camera_shift in the state) and the first time it passes CameraShiftWatch.WARN_PX it is said."""
        if getattr(self.a, "no_shift_watch", False) or self.feed is None or self.tracker is None or self.board is None:
            return
        if getattr(self, "shift_watch", None) is not None:
            return
        b = self.board
        self.shift_warned = False

        def on_shift(d):
            lst = self.st.setdefault("camera_shift", [])
            lst.append([d["t"], d["du"], d["dv"], d["cc"]])
            del lst[:-3000]
            if d["warn"] and not self.shift_warned:
                self.shift_warned = True
                self.say("  The camera's picture has moved %.1f px since the runs started (its mount creeping?): "
                         "positions are corrected for it from here on." % math.hypot(d["du"], d["dv"]))
                self.mark("e02 camera moved %.1f px" % math.hypot(d["du"], d["dv"]))
        self.shift_watch = CAM.CameraShiftWatch(self.cam, (b.x0, b.y0, b.x1, b.y1), self.tracker, on_shift=on_shift)
        self.feed.hooks.append(self.shift_watch)

    MARK_ACCEPT_CM = 2.5    # chosen: the robots' camera calibration is kept only if the tape marks put it within
    # this (median); 2026-10-24 dry run with the robots scattered: explore passed its own checks at 4.3 cm off the
    # marks, and the session that followed lost robots 29 times. DOE: explore's mark error against the runs' errors

    def explore_tape_check(self, names):
        """With --marks: right after explore, the tape-mark check judges the robots' own camera calibration. Too far
        off the marks: explore's calibration is put aside (the fallback, said plainly) and the marks are taken again
        with the calibration that replaces it."""
        if not getattr(self.a, "marks", False):
            return
        src = (self.st["setup"].get("calibration_source") or {})
        if not src.get("session"):
            return                                # explore already fell back: the session's check does the marks
        self.mark_check()
        d = sorted(math.hypot(s_["dx"], s_["dy"]) for s_ in self.camerr.samples if s_.get("source") == "mark")
        if len(d) < 3:
            return
        med = d[len(d) // 2]
        self.st["setup"]["explore_tape_check"] = dict(median_cm=round(med, 2), n=len(d), limit_cm=self.MARK_ACCEPT_CM)
        limit = getattr(self.a, "mark_accept_cm", None) or self.MARK_ACCEPT_CM
        self.st["setup"]["explore_tape_check"]["limit_cm"] = limit
        if med <= limit:
            self.say("  The robots' camera calibration agrees with the tape marks within %.1f cm (median): kept." % med)
            return
        self.camerr.samples = [s_ for s_ in self.camerr.samples if s_.get("source") != "mark"]
        self.st["camera_error_samples"] = self.camerr.to_list()
        self.st["setup"]["marks_done"] = None
        self.explore_fallback(names, "the tape marks put the robots' calibration %.1f cm off (median; limit %.1f)"
                              % (med, limit))
        self.mark_check()

    def session(self):
        st = self.st
        self.start_shift_watch()
        if getattr(self.a, "marks", False) and not st["setup"].get("marks_done"):
            self.mark_check()
        if not st["schedule"]:
            sched = self.planned_schedule()
            if st["setup"].get("square_left_out"):
                sched = [r for r in sched if r["route"] != 3]
                for i, r in enumerate(sched):
                    r["order"] = i + 1
            st["schedule"] = sched
            st.save()
            for line in PL.plan_words(sched, self.a.run_s, self.a.find_s, 0):
                self.say(line)
        self.write_runs()
        st["t_session_start"] = time.time()
        self.in_runs = True
        if getattr(self.a, "own_sensor_only", False):
            self.skip_conds |= {"K", "KC"}
            self.say("Own-sensor runs only (--own-sensor-only): the K and KC runs stay in the plan for later.")
        self.mark("e02 session start")
        n_this = 0
        cur_block = None
        while True:
            r = PL.next_run(st["schedule"], lambda rb: self.fitted(rb) is not None, self.unavailable,
                            self.skip_conds)
            if r is None:
                break
            if self.wrap_up:
                # the hard cap (e02_timeout.py): no new run; the last one finished and parked
                c_ = self.clock
                self.note_end(TO.CAP, c_.why(TO.CAP) if c_ is not None else "hard cap", parked=True)
                self.say("Hard cap: no new run starts; the robots are parked after their last run.")
                break
            if self.a.max_runs and n_this >= self.a.max_runs:
                self.say("Stopping after %d runs (--max-runs)." % n_this)
                break
            if r["block"] != cur_block:
                if cur_block is not None:
                    el = time.time() - st["t_session_start"] + st.get("elapsed_before", 0)
                    need = st.mean_run_s() * sum(1 for x in st["schedule"] if x["block"] == r["block"])
                    if not self.a.dry_run and el + need > self.a.hours * 3600 + 15 * 60:
                        self.say("Time is up for whole blocks (%s in; the next block needs about %s). The remaining "
                                 "blocks stay in the plan: --resume runs them later." % (hms(el), hms(need)))
                        break
                cur_block = r["block"]
                self.say("--- Block %d ---" % cur_block)
                self.recorder_ok()
                self.boards_still()
            elif self.marks_unconfirmed >= 3:
                self.recorder_ok()
            self.do_run(r)
            n_this += 1
            for c in ("K", "KC"):
                bad = [x for x in st["schedule"] if x["cond"] == c and x["status"] == "failed"
                       and x.get("t_end", x.get("t_start", 0)) >= st["t_session_start"]]
                if c not in self.skip_conds and len(bad) >= COND_FAIL_N:
                    # backup: the camera-dependent runs keep failing; the runs on the robots' own sensors still give
                    # accuracy data (the camera only measures their start and end)
                    self.skip_conds.add(c)
                    self.mark("e02 skip condition %s" % c)
                    self.say("  %d %s runs (%s) failed this session: the rest of them stay in the plan for later; the "
                             "other conditions carry on." % (len(bad), c, PL.COND_WORDS[c]))
            if not [x for x in st["schedule"] if x["status"] == "pending" and x["robot"] not in self.unavailable
                    and x["cond"] not in self.skip_conds]:
                if self.skip_conds or self.unavailable:
                    self.say("Nothing left that can run now (put aside: %s). --resume runs the rest later." % ", ".join(
                        sorted(self.skip_conds) + sorted(self.unavailable)))
        if self.return_after:
            self.send_back_aside()
        done = [r for r in st["schedule"] if r["status"] == "done"]
        self.mark("e02 session end")
        self.say("=== Session: %d runs done, %d failed, %d still to do ===" % (
            len(done), sum(1 for r in st["schedule"] if r["status"] == "failed"),
            sum(1 for r in st["schedule"] if r["status"] == "pending")))
        self.say("  Sensor trust (learned, e02_trust; %s):" % os.path.basename(self.trust_path))
        for line in self.trust.summary([n for n in st["robots"]]):
            self.say("    " + line)
        if self.degrade:
            self.say("  " + self.degrade.words())
        st["sensor_trust_summary"] = self.trust.summary([n for n in st["robots"]])
        et = st.get("edge_touches") or []
        for a in ("x", "y"):
            v = [r[4] - r[3] for r in et if r[2] == a]
            if v:
                self.say("  Open edge %s: felt %+.1f cm from the map's line (median of %d edge stops)"
                         % (a, float(np.median(v)), len(v)))
        for u in st.get("open_edge_updates") or []:
            self.say("  Open edge %s moved %.2f -> %.2f cm (%d stops)" % (u["edge"], u["before"], u["after"], u["touches"]))
        if st["setup"].get("routes_left_out"):
            self.say("  Routes left out after the map changed: %s" % ", ".join(
                PL.ROUTE_NAMES[r] for r in st["setup"]["routes_left_out"]))
        qrows = self.quality.rows()
        if qrows:
            write_csv(os.path.join(self.dir, "quality_map.csv"), list(qrows[0].keys()), qrows)
            poor = [r for r in qrows if r["poor"]]
            self.say("  Camera quality: %d of %d board cells seen enough; %d poor (the robot shows little detail or the "
                     "lock-on rarely holds: quality_map.csv)" % (sum(1 for r in qrows if r["n"] >= CAM.QualityMap.MIN_N),
                                                                 len(qrows), len(poor)))
        sh = st.get("camera_shift") or []
        if sh:
            mx = max(math.hypot(q[1], q[2]) for q in sh)
            self.say("  The camera's picture moved at most %.1f px during the runs (corrected; %d checks)" % (mx, len(sh)))
        if self.camerr.samples:
            for src, d in sorted(self.camerr.summary().items()):
                self.say("  The camera against %s: %.1f cm off at the median, %.1f at most (%d)" % (
                    "the tape marks" if src == "mark" else "the finds", d["median_cm"], d["max_cm"], d["n"]))
        kp = os.path.join(self.dir, "kc_stops.csv")
        if os.path.exists(kp):
            with open(kp) as fh:
                kr = list(csv.DictReader(fh))
            if kr:
                self.say("  KC: %d stops fixed; weight towards the camera median %.2f (one fix per stop)" % (
                    len(kr), float(np.median([float(r["weight"]) for r in kr]))))
        if st.get("board_line_updates"):
            last = {u["board"]: u for u in st["board_line_updates"]}
            for b, u in sorted(last.items()):
                self.say("  The %s-board's line ended at %.2f cm (the picture said %s): moved to where the robots feel "
                         "it" % (b, u["after"], u["picture"]))
        if st.get("confidence_pauses"):
            cp = st["confidence_pauses"]
            self.say("  Confidence pauses: %d (%d with a camera fix; threshold %.1f cm)" % (
                len(cp), sum(1 for r in cp if r[5].startswith("paused for")), CONF_PAUSE_CM))
        if st.get("board_corrections"):
            self.say("  Pictures late: the front sensor on a board corrected a robot %d times" % len(st["board_corrections"]))
        self.say("  Camera calibration for every run: %s" % (
            ((st.get("setup") or {}).get("calibration_source") or {}).get("words") or st.get("calibration") or "?"))
        for cond in PL.CONDITIONS:
            e = [r["end_err_cm"] for r in done if r["cond"] == cond and r.get("end_err_cm") is not None]
            f = [r["far_err_cm"] for r in done if r["cond"] == cond and r.get("far_err_cm") is not None]
            if e:
                self.say("  %-2s (%s): %d runs, end error mean %.1f cm, far point mean %.1f cm (camera; the analysis "
                         "does the statistics)" % (cond, PL.COND_WORDS[cond], len(e), float(np.mean(e)),
                                                  float(np.mean(f)) if f else float("nan")))
        for robot in st["robots"]:
            self.say("  %s: %s" % (robot, FIT.describe(self.fitted(robot))))
        st["elapsed_before"] = st.get("elapsed_before", 0) + time.time() - st["t_session_start"]
        st.save()


def stream_url(a):
    """The overhead camera's live stream; --stream-q lowers its JPEG quality (fewer bytes over WiFi, more pictures a
    second: 2026-10-22, the laptop got only ~2 a second at the default quality 95)."""
    q = getattr(a, "stream_q", 0)
    return "%s:%d/live%s" % (a.pi, a.overhead_port, ("?q=%d" % q) if q else "")


KC_WP_MAX_CM = 15.0      # chosen: a still camera position this near a route waypoint is the robot standing there


def kc_belief(wps, cam_xy, i0):
    """Where the robot believes it stands at a KC stop: the route waypoint (index >= i0: it only goes forward along
    its route) nearest the camera's position, if within KC_WP_MAX_CM. Returns (index, (x, y), cm driven since
    waypoint i0, deg turned since then) or None."""
    if not wps:
        return None
    best = None
    for i in range(i0, len(wps)):
        d = math.hypot(wps[i][0] - cam_xy[0], wps[i][1] - cam_xy[1])
        if best is None or d < best[0] - 1e-9:
            best = (d, i)
    if best is None or best[0] > KC_WP_MAX_CM:
        return None
    i = best[1]
    dist = turned = 0.0
    for k in range(i0, i):
        dist += math.hypot(wps[k + 1][0] - wps[k][0], wps[k + 1][1] - wps[k][1])
    for k in range(max(i0, 1), i + 1):
        if k + 1 < len(wps) and k - 1 >= 0:
            h1 = math.atan2(wps[k][1] - wps[k - 1][1], wps[k][0] - wps[k - 1][0])
            h2 = math.atan2(wps[k + 1][1] - wps[k][1], wps[k + 1][0] - wps[k][0])
            turned += abs(math.degrees(math.atan2(math.sin(h2 - h1), math.cos(h2 - h1))))
    if i0 == 0 and i == 0:
        dist, turned = 2.0, 90.0      # at the start: since its find (a back-off and a turn or two)
    return i, (float(wps[i][0]), float(wps[i][1])), dist, turned


def kc_fix_from(samples, after_stop_s=0.1, min_still_s=0.3, moved_cm=0.8):
    """A KC camera fix only from pictures taken after the robot stopped: (x, y, t of the newest picture, t_stop,
    the stretch before the stop) or None. t_stop = the last picture that was more than moved_cm from where it
    stands now; the position = the median of the pictures from t_stop + after_stop_s on, at least min_still_s of
    them (so the firmware's own still check, e02_fix_still_ms, and the picture agree)."""
    if not samples:
        return None
    sp = CAM.still_position(samples, min_s=min_still_s)
    if sp is None:
        return None
    moving = [q for q in samples if math.hypot(q[1] - sp[0], q[2] - sp[1]) > moved_cm]
    t_stop = moving[-1][0] if moving else samples[0][0]
    if not moving and samples[-1][0] - samples[0][0] < 2.5:
        return None                               # it may have stopped just before this window: not sure when
    still = [q for q in samples if q[0] >= t_stop + after_stop_s]
    if len(still) < 3 or still[-1][0] - still[0][0] < min_still_s:
        return None
    x = float(np.median([q[1] for q in still]))
    y = float(np.median([q[2] for q in still]))
    seg = [q for q in samples if t_stop - 1.0 <= q[0] <= t_stop] if moving else []
    return x, y, still[-1][0], t_stop, seg


def _r(v):
    return "" if v is None else round(v, 5)


def write_csv(path, cols, rows):
    tmp = path + ".tmp"
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    try:
        os.replace(tmp, path)
    except OSError:     # open in Excel: keep the copy next to it
        pass


FIND_HOLD_MAX = 3   # holds for one find before it counts as a failed find (the run is redone later); Nick's time, no DOE


def find_hold_kind(result):
    """'no board seen' / 'no board behind' when a backing-in find (firmware 2026-10-25b) found no board, else None."""
    r = (result or "").lower()
    for k in ("no board seen", "no board behind"):
        if k in r:
            return k
    return None


def find_hold_words(robot, result, where, board):
    """The find hold's words for Nick (Nick, 2026-10-25): (why: what happened, one line for the page's big text;
    detail: which board, where the robot is, what to check, and what each answer does)."""
    kind = find_hold_kind(result) or "no board"
    m = re.search(r"\b([xy]) board", result or "")
    which = m.group(1) if m else None
    bname = ("the %s-board" % which) if which else "a board"
    nums = [float(v) for v in re.findall(r"(\d+(?:\.\d+)?) cm", result or "")]
    if kind == "no board seen":
        seen = nums[0] if nums else None
        exp_ = re.search(r"should be (\d+(?:\.\d+)?)", result or "")
        why = "its find stopped before backing in: facing %s, its front sensor %s where the board should be %s" % (
            bname, "saw nothing nearer than %.0f cm" % seen if seen is not None else "did not see it",
            "%.0f cm away" % float(exp_.group(1)) if exp_ else "close")
    else:
        why = ("its find stopped while backing into %s: it backed as far as it is allowed%s and neither back switch "
               "touched a board" % (bname, " (%.0f cm)" % nums[0] if nums else ""))
    line = None
    if board is not None and which:
        line = board.xboard_cm if which == "x" else board.yboard_cm
    parts = []
    if where is not None:
        x, y, h, src = where
        here = "about (%.0f, %.0f) cm by %s" % (x, y, src)
        if board is not None:
            here += ", %.0f cm from the x-board's line and %.0f cm from the y-board's line" % (
                abs(y - board.xboard_cm), abs(x - board.yboard_cm))
        if h is not None:
            here += ", facing %.0f deg" % h
        parts.append("Where %s is: %s." % (robot, here))
    else:
        parts.append("Where %s is: not known (the camera does not see it now); it is near %s." % (robot, bname))
    checks = ["is %s there, standing on the cement and pressed against the side of the plywood%s?" % (
        bname, " (its face at %s = %.0f cm on the camera's map)" % ("y" if which == "x" else "x", line)
        if line is not None else ""),
        "is anything between %s and %s (another robot, a brick, a cable, a hand)?" % (robot, bname)]
    if kind == "no board seen":
        checks.append("is the board upright, not leaning away from the plywood?")
    else:
        checks.append("do both of %s's back switches click when you press them by hand, and is nothing caught on its "
                      "back bumper?" % robot)
    parts.append("Check: " + " ".join("(%d) %s" % (i + 1, c) for i, c in enumerate(checks)))
    parts.append("Answers: Carry on = %s tries the find again after a fresh camera fix (put the board back first if "
                 "it was missing). The board is gone = %s finds itself against any two edges instead (the edge "
                 "find), and the map marks %s missing. Leave %s out = it stays where it is for the rest of the "
                 "session. Stop = the session stops (--resume later)." % (robot, robot, bname, robot))
    return why, " ".join(parts)


def check_params(watch, run, cmds, wait_s=2.0):
    """run_sheet.check_params, with `corr reset` understood."""
    import run_sheet
    return run_sheet.check_params(watch, run, cmds, wait_s)


class NamingWatch:
    """tools/live's blink tracker on the overhead frames during the naming check (record only)."""

    def __init__(self, cam, folder):
        for p in (os.path.join(TOOLS, "live"), os.path.join(TOOLS, "blink"), os.path.join(TOOLS, "fusion")):
            if p not in sys.path:
                sys.path.insert(0, p)
        import track_live as TL
        calib = CAM.write_floor_calibration(cam, os.path.join(folder, "floor_calibration_for_blink.json"))
        ns = argparse.Namespace(led_height_cm=None, names=None, vmax_cms=40.0, exposure_ms=None, replay=None,
                                name_p=0.9, no_fast=False, calib_results=None, led_offset_cm=None)
        import floormap
        ns.led_height_cm = floormap.LED_HEIGHT_CM
        self.tracker, self.fmap = TL.make_tracker(ns, calib)
        self.fixes = defaultdict(list)
        self.lock = threading.Lock()
        self.out = open(os.path.join(folder, "naming_check_fixes.jsonl"), "w")

    def push(self, img, t):
        try:
            msgs = self.tracker.push(img, t)
        except Exception:
            return
        with self.lock:
            for m in msgs:
                if m.get("type") == "fix":
                    self.fixes[m["robot"]].append((m["x"], m["y"]))
                    self.out.write(json.dumps(m) + "\n")

    def summary(self, known):
        out = {}
        with self.lock:
            for n, kp in known.items():
                fx = self.fixes.get(n, [])
                near = [f for f in fx if kp and math.hypot(f[0] - kp[0], f[1] - kp[1]) <= 8.0]
                wrong = len(fx) - len(near) if kp else None
                words = ("named %d times, %d of them at its spot" % (len(fx), len(near))) if fx else "never named"
                out[n] = dict(fixes=len(fx), at_its_spot=len(near), elsewhere=wrong, words=words)
            others = {k: len(v) for k, v in self.fixes.items() if k not in known}
        self.out.close()
        return dict(robots=out, other_names=others)


# =============================================================================== main
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pi", default="st1maxfini.local", help="the Pi recorder's address")
    ap.add_argument("--overhead-port", type=int, default=8000)
    ap.add_argument("--side-port", type=int, default=8001)
    ap.add_argument("--stream-q", type=int, default=0, help="JPEG quality of the live camera stream (0 = the Pi's "
                                                              "default, 95; e.g. 70 for more pictures a second)")
    ap.add_argument("--no-front-steer", action="store_true",
                    help="the front sensor is only logged, never makes the runner find a robot again (if its first "
                         "real readings turn out noisy)")
    ap.add_argument("--no-face-reading", action="store_true",
                    help="no front-sensor reading right after a nose-first find (face_reading; on by default)")
    ap.add_argument("--no-front-learn", action="store_true",
                    help="use the fixed front-sensor offset and tolerance (FRONT_OFFSET_CM / FRONT_TOL_CM) instead of "
                         "each robot's, learned every session (e02_front.py)")
    ap.add_argument("--front-learn", action="store_true",
                    help="learn the front sensor's offset and spread against the map in this session (from scratch); "
                         "off: the values from data (FRONT_OFFSET_CM, FRONT_TOL_CM)")
    ap.add_argument("--no-unc-learn", action="store_true",
                    help="keep the uncertainty model's starting numbers (e02.py UNC_*) instead of learning each robot's "
                         "from its own camera corrections every session (e02_unc.py)")
    ap.add_argument("--no-side", action="store_true", help="no side camera this session: don't check or ask about it")
    ap.add_argument("--map", action="store_true", help="after setup, one robot maps the board by touch (about 6 min)")
    ap.add_argument("--map-only", action="store_true", help="setup and the mapping only, then stop")
    ap.add_argument("--bricks", action="store_true", help="after the session: the brick demo (you put bricks or any "
                                                         "objects down, a robot crosses round them three times: factory "
                                                         "settings, then its calibration, then with camera fixes)")
    ap.add_argument("--bricks-only", action="store_true", help="setup (and the naming check: the gyro creep), then the "
                                                              "brick demo only")
    ap.add_argument("--tight-gap", action="store_true", help="the brick demo (after the session, or with "
                                                             "--bricks-only) goes straight through the narrowest gap "
                                                             "between two objects the robot can safely make, from its "
                                                             "measured precision this session")
    ap.add_argument("--gap-k", type=float, default=None, help="tight gap: room = half the robot + k x the 95th "
                                                              "percentile of its measured error (default 1.0)")
    ap.add_argument("--dry-explore-fail", action="store_true", help="dry run: explore's fit check always fails "
                    "(tests the fallback to a calibration on hand)")
    ap.add_argument("--dry-explore-only", action="store_true", help="dry run: end the session as soon as explore "
                    "has its result (found the board, or failed: no fallback); for explore-only batches (2026-10-26 "
                    "explore reliability study)")
    ap.add_argument("--dry-remove-board", default="", help="dry run: the remove-a-board test: 'x@600' takes the x "
                    "board away 600 s after the start, without telling the runner")
    ap.add_argument("--dry-bricks", default="", help="dry run: the bricks the simulator puts down when asked, "
                                                     "'x,y,length,width[,deg];...' cm")
    ap.add_argument("--dance", action="store_true", help="after the session: the 3-robot dance to a beat (about 1.5 "
                                                        "min; watch and listen on http://localhost:8090/dance)")
    ap.add_argument("--dance-only", action="store_true", help="setup (and the naming check), then the dance only")
    ap.add_argument("--dance-bpm", type=float, default=120.0, help="the dance's tempo (120: the lights flash on "
                                                                   "every beat)")
    ap.add_argument("--view-port", type=int, default=8090, help="live view in a browser: http://localhost:<port> (0 = off)")
    ap.add_argument("--view-open", action="store_true",
                    help="the live view can be opened from other computers and phones on the WiFi (the Pi run)")
    ap.add_argument("--web-ask", action="store_true",
                    help="questions go on the live page (answered from any phone or computer); no answer in "
                         "WEB_ASK_TIMEOUT_S = the default (the Pi run)")
    ap.add_argument("--on-pi", action="store_true",
                    help="the whole session runs on the Pi (run-e02-pi.sh): the camera is this computer "
                         "(--pi 127.0.0.1), the live view is open on the WiFi, questions go on it")
    ap.add_argument("--map-rays", type=int, default=16)
    ap.add_argument("--map-robot", default="", help="which robot maps (default: the first)")
    ap.add_argument("--dry-kept-pose", action="store_true", help="dry run: the robots kept a board pose from an earlier "
                    "session (the 2026-10-24 stage-3 case)")
    ap.add_argument("--dry-start", default="parks", choices=["parks", "swapped", "scatter", "corners", "edge", "s1510", "s1545", "s2114"],
                    help="dry run: where the simulated robots stand at the start")
    ap.add_argument("--calib", help="floor calibration file (default: the studio project's cameras/overhead.json)")
    ap.add_argument("--project", help="the studio project folder (default Documents\\recognition-project)")
    ap.add_argument("--make-calibration", action="store_true", help="make one from the board corners even if one exists")
    ap.add_argument("--lens-height-cm", type=float, help="camera lens height above the floor, if measured")
    ap.add_argument("--board-sides", help="which edges are blocked, as x-board side,y-board side (0 = low, 1 = high "
                                          "end of the other axis; e.g. 0,0 = boards along y = 0 and x = 0). Default: "
                                          "asked, with the camera picture")
    ap.add_argument("--robots", default="Left,Center,Right")
    ap.add_argument("--robot", action="append", default=[], help="robot IP, if robots aren't found automatically")
    ap.add_argument("--hours", type=float, default=2.0)
    # session timeouts (Nick, 2026-10-26; e02_timeout.py). DOE note: chosen convenience settings, not measured; they
    # only decide how long an unattended session waits for an answer, so no DOE is needed for them.
    ap.add_argument("--answer-timeout-min", type=float, default=TO.ANSWER_TIMEOUT_MIN,
                    help="a hold or question nobody answers for this long ends the session: every robot stopped, "
                         "saved, uploaded (default %g; 0 = off). Questions that take their own default after a time "
                         "limit keep it" % TO.ANSWER_TIMEOUT_MIN)
    ap.add_argument("--after-end-timeout-min", type=float, default=TO.AFTER_END_TIMEOUT_MIN,
                    help="after the planned time (--hours from the start) is up, a hold or question nobody answers for "
                         "this long ends the session (default %g; 0 = off)" % TO.AFTER_END_TIMEOUT_MIN)
    ap.add_argument("--hard-cap-min", type=float, default=TO.HARD_CAP_MIN,
                    help="hard cap at --hours plus this: no new run starts, the current one finishes and parks if it "
                         "can within %d s, else every robot is stopped; then the session ends and uploads (default "
                         "%g; 0 = off)" % (TO.WRAP_GRACE_S, TO.HARD_CAP_MIN))
    ap.add_argument("--run-s", type=float, default=PL.RUN_S, help="time a route run with its parking takes, for the "
                                                                   "plan (s; measured ~70)")
    ap.add_argument("--find-s", type=float, default=PL.FIND_S, help="time a find takes, for the plan (s)")
    ap.add_argument("--help-wait-min", type=float, default=10.0, help="how long to wait for Nick before putting a "
                                                                       "robot aside (minutes)")
    ap.add_argument("--blocks", type=int, help="regular blocks after the all-U block (default: what fits in --hours)")
    ap.add_argument("--routes", default="1,2,3", help="only these routes (testing)")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--fix-hz", type=float, default=FIX_HZ)
    ap.add_argument("--mark-accept-cm", type=float, default=0.0,
                    help="the tape marks keep the robots' camera calibration only within this (median; 0 = the "
                         "default, Runner.MARK_ACCEPT_CM)")
    ap.add_argument("--degrade", default="", help="make a sensor bad on purpose, in software (e02_degrade.py), e.g. "
                    "camera:noise=3 or wheels:jitter=0.05")
    ap.add_argument("--trust-file", default=None, help="the learned sensor trust (default logs/sensor_trust.json)")
    ap.add_argument("--sleep-idle", type=int, default=0, help="robots switched on but not in this session go to sleep, "
                    "checking in every this many minutes (firmware 2026-10-25a; 0: off)")
    ap.add_argument("--battery-watch-s", type=float, default=300,
                    help="every this many seconds, log every switched-on robot's battery to battery.csv, driving or "
                         "idle (0: off; on by default, Nick 2026-10-24: weeks of it decide the drain questions)")
    ap.add_argument("--no-shift-watch", action="store_true",
                    help="do not measure or correct the camera's creep during the runs (CAM.CameraShiftWatch)")
    ap.add_argument("--load", default=None,
                    help="a planned background load from the first run on, switched on and off in blocks (e02_load.py): "
                         "cpu[:N], disk:FOLDER (read and compress its files, like an upload), or cmd:COMMAND")
    ap.add_argument("--load-on-s", type=float, default=300.0, help="--load: seconds on (default 300)")
    ap.add_argument("--load-off-s", type=float, default=300.0, help="--load: seconds off (default 300)")
    ap.add_argument("--home", action="store_true",
                    help="Nick is home: the virtual border sits closer to the edge (camera %g cm, the robots' own %g cm; "
                         "away: %g / %g); a crossing still stops every robot and asks" % (
                             HOME_BORDER_CM, HOME_FW_BORDER_CM, BORDER_CM, FW_BORDER_CM))
    ap.add_argument("--marks", action="store_true",
                    help="before the runs, a tape-mark check: Nick puts a robot by hand on each tape mark "
                         "(e02_camerr.MARKS), and the camera's error there corrects KC's fixes and the far-point scores")
    ap.add_argument("--recover-cm", "--lost-cm", dest="recover_cm", type=float, default=ST.RECOVER_CM,
                    help="a run that ends this far from its start spot needed recovery (noted, not an outlier)")
    ap.add_argument("--naming-s", type=int, default=120)
    ap.add_argument("--skip-naming", action="store_true")
    ap.add_argument("--skip-find-check", action="store_true")
    ap.add_argument("--max-runs", type=int, default=0, help="stop after this many runs (0 = all)")
    ap.add_argument("--classic-setup", action="store_true", help="the old setup: a calibration file (green outline "
                    "check) and the two board questions, instead of the robots exploring")
    ap.add_argument("--no-recording", action="store_true", help="the Pi runs camrec.py --no-record (live picture "
                    "only): no video and no notes are expected; the runs are measured from the live picture, and "
                    "snapshots/ keeps each run's measured frames")
    ap.add_argument("--edge-careful", action="store_true", help="production behaviour: never turn on the spot near "
                    "an open edge with the heading unknown (call Nick instead); test sessions turn anyway and log it")
    ap.add_argument("--edge-find", action="store_true", help="every find against any two boundaries (boards or open "
                    "edges, no camera) instead of `run find` against the two boards; also used on its own once a "
                    "board is found missing")
    ap.add_argument("--no-boards", action="store_true", help="no boards at all: all four sides are open edges from "
                    "the start (Nick, 2026-10-26 22:49: the boards taken off for edge data; Monday's school table has "
                    "none). Implies --classic-setup (explore assumes two boards), --edge-find and --skip-find-check")
    ap.add_argument("--switches-back", nargs="?", const="all", default=None, metavar="ROBOTS",
                    help="the robots wear the one attachment with the switches at the BACK (Nick, 2026-10-25): sets "
                         "sw_on 1, sw_back 1, e02_find_reverse 1, e02_back_face_cm 6.10 (read back), so every find "
                         "backs into the boards; firmware 2026-10-25b. All robots, or a list: --switches-back Left,Center")
    ap.add_argument("--no-lock", action="store_true", help="backup: the tracker without the lock-on (the motion "
                    "detector alone, as before 2026-10-23)")
    ap.add_argument("--own-sensor-only", action="store_true", help="backup: only the runs where the robots drive on "
                    "their own sensors (U, C); K and KC stay in the plan for later")
    ap.add_argument("--resume", nargs="?", const="latest", help="carry on a session (folder, or the latest)")
    ap.add_argument("--new", action="store_true", help="a new session: don't offer to carry on an unfinished one "
                                                       "(the start page, tools/pi/launcher.py, chose already)")
    ap.add_argument("--auto", action="store_true", help="answer every question with its default (no person)")
    ap.add_argument("--fit-max-rms-cm", type=float, default=None,
                    help="the self-calibration's camera-vs-wheels limit per stretch (default e02_fit.MAX_RMS_CM, 1.5; "
                         "2026-10-24 Pi plan: the session-1 calibration file gave 2.1-2.5 and every fit was refused)")
    ap.add_argument("--dry-bricks-now", action="store_true", help="dry run: the --dry-bricks objects stand there from "
                    "the start (e.g. one just past an open edge), not only in the brick demo")
    ap.add_argument("--dry-scene", default="", help="dry run: events during explore, ROBOT:KIND:N:WHAT[:AT_S],... "
                    "(e.g. Right:ray:2:o = a hold during Right's 2nd ray, answered o; Left:bigturn:1:gap3 = the "
                    "camera loses Left for 3 s in its first 175 deg turn; Right:ray:2:none = a hold nobody answers, the session "
                    "timeouts' test): the 15:10 re-creation")
    ap.add_argument("--dry-turn-fault", default="", help="dry run: ROBOT[@S]: that robot's turns break S s (default 5) "
                    "after its compass check: its gyro reads every turn the wrong way, so turns run to the timeout and "
                    "straight drives curve (the 15:45 Right re-creation)")
    ap.add_argument("--dry-hold-answer", default="c", choices=["c", "a", "o"],
                    help="dry run with --auto: the answer every hold gets (default c; a = the 15:45 session's answer)")
    ap.add_argument("--dry-camera-fps", type=float, default=None, help="dry run: the simulated camera's pictures a "
                    "second (default the simulator's 15)")
    ap.add_argument("--dry-guard-shift", default="", help="dry run: DX/DY cm: the explore border's calibration off the "
                    "simulator's floor (e.g. -9/-9 leaves edge stops' centres past the line, as at 15:10)")
    ap.add_argument("--dry-slow-camera", default="", help="dry run: PERIOD/ON/MS[/FIRST]: every PERIOD s, for ON s, each "
                    "picture MS ms slower, from FIRST s on (the speed guard's test)")
    ap.add_argument("--no-stall-watch", action="store_true",
                    help="do not record what blocks the camera loop (e02_stall.py: stalls.csv, threads.csv)")
    ap.add_argument("--fits-in-runner", action="store_true",
                    help="run explore's camera fits in the runner's own process, as before 2026-10-25 (for the "
                         "before/after measurement; default: a helper process, e02_offload.py)")
    ap.add_argument("--plain-commands", action="store_true", help="backup: commands as before 2026-10-26 (no numbers, "
                    "no acknowledgments, no settings read back, no link gauge)")
    ap.add_argument("--dry-drop-pct", type=float, default=0.0, help="dry run: each command to a robot lost at random "
                    "with this chance, %% (0-15; the 2026-10-25 21:14 link lost about 10); the robots' lines a quarter "
                    "as often")
    ap.add_argument("--dry-drop-first", default="", help="dry run: ROBOT:TEXT:N,...: the first N commands to ROBOT "
                    "containing TEXT are lost (the 21:14 lost back-out: Right:go -:1)")
    ap.add_argument("--dry-low-contrast", default="", help="dry run: ROBOT[@FROM-TO],...: that robot drawn washed out "
                    "(as bright as the plywood) from FROM to TO s after the simulator starts (2026-10-26 10:00, Left)")
    ap.add_argument("--dry-run", action="store_true", help="against the simulator (tools/experiment/e02_sim.py)")
    ap.add_argument("--dry-true-calib", action="store_true", help="dry run: use the simulator's true camera "
                                                                  "instead of making one from the corners")
    TKW.add_args(ap)
    a = TKW.fix_args(ap.parse_args(argv))
    if a.load:
        try:
            LD.parse(a.load)
        except ValueError as e:
            ap.error(str(e))
    if a.fit_max_rms_cm is not None:
        # chosen for the session (Nick 2026-10-24: 2.5 cm); the sensor-trust build (or a DOE) should set it
        FIT.MAX_RMS_CM = a.fit_max_rms_cm
    if a.on_pi:
        # the Pi run (Nick, 2026-10-24: start a session and leave the house): the camera is this computer, the
        # live view is open on the WiFi and questions are answered there (run-e02-pi.sh starts all of it)
        a.pi = "127.0.0.1"
        a.view_open = True
        a.web_ask = True

    sim = None
    if not a.resume and not a.dry_run and not a.auto and not a.new:
        # an unfinished real session on this laptop: offer to carry it on (the launcher double-click path)
        for c in sorted(glob.glob(os.path.join(REPO, "logs", "e02-*", "e02_state.json")), reverse=True):
            if "-dryrun" in c:
                continue
            try:
                left = sum(1 for r in json.load(open(c)).get("schedule", []) if r.get("status") in ("pending", "running"))
            except (OSError, ValueError):
                continue
            if left:
                ans = input("An unfinished E02 session is on this laptop (%s, %d runs to go). Carry it on? "
                            "Type y and press Enter to carry on, or n for a new session: " % (os.path.dirname(c), left))
                if ans.strip().lower().startswith("y"):
                    a.resume = os.path.dirname(c)
            break
    if a.resume:
        if a.resume == "latest":
            cands = sorted(c for c in glob.glob(os.path.join(REPO, "logs", "e02-*", "e02_state.json"))
                           if a.dry_run == ("-dryrun" in os.path.basename(os.path.dirname(c))))
            if not cands:
                sys.exit("No E02 session to resume in logs/.")
            sdir = os.path.dirname(cands[-1])
        else:
            sdir = a.resume
        state = PL.State.load(sdir)
        print("Resuming %s" % sdir)
        a.dry_run = a.dry_run or state["setup"].get("dry_run", False)
    else:
        sdir = os.path.join(REPO, "logs", "e02-" + dt.datetime.now().strftime("%Y%m%d-%H%M%S")
                            + ("-dryrun" if a.dry_run else ""))
        os.makedirs(sdir, exist_ok=True)
        state = PL.State(sdir)
        state["robots"] = [n.strip() for n in a.robots.split(",") if n.strip()]
        state["seed"] = a.seed if a.seed is not None else int(time.time()) % 100000
        state["setup"]["dry_run"] = a.dry_run
        state["setup"]["args"] = vars(a)
        state.save()
    if getattr(a, "no_boards", False):
        a.classic_setup = True
        a.edge_find = True
        a.skip_find_check = True
    if a.dry_run:
        a.auto = not a.web_ask            # --web-ask in a dry run: the questions go on the live page (tests it)
        a.pi = "127.0.0.1"
        a.robot = a.robot or ["127.0.0.%d" % (2 + k) for k in range(len(state["robots"]))]
        if a.board_sides is None:
            a.board_sides = "1,1"          # Nick's layout: boards along x = max and y = max (far corner)
        sim = start_sim(state, sdir)
        if a.dry_true_calib and not state.get("calibration"):
            a.calib = os.path.join(sdir, "sim", "camera_model.json")
        if not a.calib and not state.get("calibration"):
            a.make_calibration = True

    try:
        return launch(a, state, sdir, sim)
    finally:
        stop_sim(sim, sdir)      # never leave the simulator running


def launch(a, state, sdir, sim):
    state["launches"] = state.get("launches", 0) + 1
    logdir = os.path.join(sdir, "robot-logs-%d" % state["launches"])
    state.setdefault("log_dirs", []).append(logdir)
    redo = state.interrupted()
    state.save()

    import alvik_console as AC
    import run_sheet
    run = Runner(a, state, sdir)
    run.say("E02 session folder: %s (launch %d)" % (sdir, state["launches"]))
    for r in redo:
        run.say("Run %d was interrupted last time: it will be redone." % r["order"])
    link = AC.Link(logdir, a.robot)
    link.session.quiet = True
    run.link = link
    run.listener = Listener(link.session)
    run.watch = run_sheet.RunWatcher()
    link.session.listeners += [run.watch, run.listener]
    if not getattr(a, "plain_commands", False):
        # 2026-10-26 runner fixes, item 1: numbered, acknowledged commands (firmware 2026-10-26a), every setting read
        # back, the link gauge (e02_link.py). --plain-commands: as before
        import e02_link as LNK
        run.link = LNK.ReliableLink(link, firmware=run.listener.firmware, folder=sdir, say=run.say)
    run.pi = run_sheet.PiNotes(a.pi)
    run.view = None
    try:
        import e02_live
        run.lab = e02_live.LiveLab()
        run.listener.lab = run.lab
    except Exception as e:
        run.lab = None
        run.say("(live lab not started: %s)" % e, quiet=True)
    if a.view_port:
        try:
            import e02_view
            run.view = e02_view.LiveView(run, a.view_port, bind="0.0.0.0" if a.view_open else "127.0.0.1")
            run.say(run.view.start())
        except Exception as e:          # the view is a convenience: never stop a session over it
            run.say("(live view not started: %s)" % e)
            run.view = None
    hist = CAM.History()
    run.history = hist
    if run.degrade.spec.get("camera"):
        run.history_raw = CAM.History()       # the true camera, for scoring only
    if not state["schedule"] and not getattr(a, "bricks_only", False) and not getattr(a, "dance_only", False):
        for line in PL.plan_words(run.planned_schedule(), a.run_s, a.find_s):
            run.say(line)
    threading.Thread(target=run.watchdog_loop, daemon=True).start()
    run.say(run.clock.words())
    state.setdefault("session_timeouts", []).append(dict(
        launch=state["launches"], t_start=round(run.clock.t_start, 1), planned_end=round(run.clock.planned_end, 1),
        answer_min=run.clock.answer_min, after_end_min=run.clock.after_end_min, cap_min=run.clock.cap_min,
        cap=None if run.clock.cap is None else round(run.clock.cap, 1)))
    threading.Thread(target=run.cap_watch, daemon=True).start()     # the hard cap (e02_timeout.py)
    code = 0
    feed = None
    try:
        if run.degrade:
            run.say("!!! " + run.degrade.words() + " (e02_degrade.py): every run's line says so.")
            state.setdefault("setup", {})["degraded"] = run.degrade.spec
        # the tracker in its own process (e02_trackproc; --tracker-in-runner: as before)
        feed = TKW.make_feed(run, a, hist, stream_url(a))
        feed.on_sample = run.on_sample
        if run.degrade.spec.get("camera"):
            feed.sample_filter = run.degrade.camera
            feed.history_raw = run.history_raw
        feed.on_perf = run.on_perf
        feed.keep_s = run.FRAMES_KEEP_S
        feed.start()
        run.feed = feed
        run.cpulog = TKW.start_cpu_log(run, a)   # cpu.csv: each core, temperature, throttling, our processes
        if getattr(a, "dry_run", False) and getattr(a, "dry_scene", ""):
            run.scene = dry_scene(a.dry_scene)
            prev_f = feed.sample_filter
            feed.sample_filter = (lambda s_, pf=prev_f: run.scene_filter(pf(s_) if pf else s_))
        if getattr(a, "dry_run", False) and getattr(a, "dry_slow_camera", ""):
            feed.hooks.append(slow_camera_hook(a.dry_slow_camera, run))
        if not getattr(a, "no_stall_watch", False):
            # what blocks the camera loop, every session (stalls.csv, threads.csv; part 2a of the 2026-10-25 fixes)
            run.stallw = STL.StallWatch(run.dir, camera_ident=lambda: getattr(run.feed, "ident", None),
                                        log=lambda m: run.say(m, quiet=True))
            run.stallw.start()
        run.setup()
        run.robots_ready()
        if (a.map or a.map_only) and not state["setup"].get("map"):
            names = [n for n in state["robots"] if n not in run.unavailable]
            if names:
                run.map_board(a.map_robot if a.map_robot in names else names[0], n_rays=a.map_rays)
        if a.map_only:
            raise SystemExit("Mapping done (--map-only): no runs this time.")
        if getattr(a, "dance_only", False):
            if not a.skip_naming and not state.get("naming_check"):
                run.naming_check()
            run.dance()
        elif getattr(a, "bricks_only", False):
            if not a.skip_naming and not state.get("naming_check"):
                run.naming_check()
            run.brick_demo()
        else:
            if not a.skip_find_check and not state["setup"].get("find_check"):
                run.find_check()
            if not a.skip_naming and not state.get("naming_check"):
                run.naming_check()
            run.session()
            if run.wrap_up and (getattr(a, "bricks", False) or getattr(a, "tight_gap", False) or getattr(a, "dance", False)):
                run.say("Hard cap: the brick demo and the dance are skipped.")
            else:
                if getattr(a, "bricks", False) or getattr(a, "tight_gap", False):
                    run.brick_demo()
                if getattr(a, "dance", False):
                    run.dance()
        run.note_end("finished", "finished")
    except KeyboardInterrupt:
        if run.timed_out:
            # a session timeout (e02_timeout.py): this is how it is meant to end, so the exit is a normal one
            run.say("Session ended by itself (%s): every robot is stopped. Start again with --resume to carry on."
                    % run.end_reason["why"])
            run.stop_everyone()
        elif run.stop_requested:
            run.note_end("nick_stop", "Nick chose stop at a hold")
            run.say("Stopped (Nick's answer at a hold): every robot is stopped. Start again with --resume to carry on.")
            code = 1
        else:
            run.note_end("ctrl_c", "Ctrl+C")
            run.say("Stopped by Ctrl+C: every robot is stopped. Start again with --resume to carry on.")
            code = 1
    except SystemExit as e:
        run.note_end("exit", str(e))
        run.say(str(e))
        code = 1
    except BaseException as e:
        run.note_end("error", "%s: %s" % (type(e).__name__, e))
        raise
    finally:
        run.ended.set()                       # no timeout interrupts the runner from here on
        run.watch_quit.set()
        if getattr(run, "load", None) is not None:
            try:
                run.load.stop()
            except Exception:
                pass
        try:
            link.close()
        except Exception:
            pass
        if feed:
            feed.close()
        if getattr(run, "cpulog", None) is not None:
            run.cpulog.stop()
        try:
            for ln_ in TKW.summary(run):
                run.say(ln_)
        except Exception as e_:
            run.say("(tracker process summary: %s)" % e_, quiet=True)
        if run.tracker is not None and getattr(run.tracker, "stats", None):
            st_ = dict(run.tracker.stats)
            for k_, v_ in (getattr(run, "tracker_stats_before", None) or {}).items():
                st_[k_] = st_.get(k_, 0) + v_          # the trackers before this one (the whole session)
            state.setdefault("tracker_stats", []).append(dict(t=round(time.time(), 1), **st_))
            n = sum(st_.get(k, 0) for k in ("both", "lock", "detector", "held", "none")) or 1
            run.say("Tracker: of the pictures, both saw the robot %.0f%%, the lock-on alone %.0f%%, the detector alone "
                    "%.0f%%, held %.0f%%, none %.0f%%; detector checks agreed %d of %d (it saw the robot), %d re-locks" % (
                        100.0 * st_.get("both", 0) / n, 100.0 * st_["lock"] / n, 100.0 * st_["detector"] / n,
                        100.0 * st_["held"] / n,
                        100.0 * st_["none"] / n, st_["agree"], st_["agree"] + st_["disagree"], st_["relocks"]))
            sh_ = dict(getattr(run.tracker, "stats_hold", None) or {})
            nh = sum(sh_.values())
            if nh:
                state["tracker_stats"][-1]["during_holds"] = sh_
                run.say("  (not counted above: %d pictures during holds, every robot stopped; the robot was seen in %.0f%% "
                        "of them)" % (nh, 100.0 * (nh - sh_.get("none", 0)) / nh))
        sw_ = getattr(run, "stallw", None)
        if sw_ is not None:
            sw_.stop()
            try:
                state["stall_summary"] = sm_ = sw_.summary()
                run.say("Camera loop blocked 100 ms or more: %d times, %.1f s in all, worst %.2f s%s" % (
                    sm_["stalls"], sm_["stall_ms_total"] / 1000.0, sm_["stall_ms_worst"] / 1000.0,
                    "; most by: " + ", ".join("%s (%.1f s)" % (k, v / 1000.0) for k, v in sm_["culprits_ms"][:3])
                    if sm_["culprits_ms"] else ""))
            except Exception as e:
                run.say("  (stall summary: %s)" % e, quiet=True)
        off_ = getattr(run, "offload", None)
        if off_ is not None:
            state["offload_calls"] = list(off_.calls)
            off_.stop()
        try:
            ps = run.perf_summary()
            if ps:
                run.say(ps)
                state["perf_summary"] = ps
            fm_ = getattr(run, "front", None)
            if fm_ is not None:
                try:
                    lims = {n: fm_.limits(n) for n in state.get("robots", {})}
                    state["front_limits"] = {n: dict(offset_cm=round(o, 2), tol_cm=round(t_, 2), source=src)
                                             for n, (o, t_, src) in lims.items()}
                    run.say("Front sensors: " + "; ".join("%s offset %+.1f cm, tolerance %.1f cm (%s)" % (n, o, t_, src)
                                                          for n, (o, t_, src) in lims.items()))
                    if not getattr(run.a, "degrade", ""):          # a degraded test never teaches the real values
                        fm_.save()
                except Exception as e:
                    run.say("  (front sensor values not saved: %s)" % e, quiet=True)
            um_ = getattr(run, "uncm", None)
            if um_ is not None:
                try:
                    sm_ = {n: um_.summary(n) for n in state.get("robots", {})}
                    state["unc_model"] = sm_
                    run.say("Own position uncertainty after 50 cm and a half turn: " + "; ".join(
                        "%s %.1f cm (%s; %d corrections this session%s)" % (
                            n, v["sd_ref_cm"], v["src"], v["corrections"],
                            "" if v["within_2_45_sd"] is None else ", %.0f%% inside 2.45 sd (95%% if right)" %
                            (100 * v["within_2_45_sd"])) for n, v in sm_.items()))
                    if not getattr(run.a, "degrade", "") and not getattr(run.a, "no_unc_learn", False):
                        um_.save()
                except Exception as e:
                    run.say("  (uncertainty model not saved: %s)" % e, quiet=True)
            sw_ = getattr(run, "speed", None)
            if sw_ is not None and sw_.spans:
                run.say("Slow spells: %d (%d with the planned load on), %.0f s in all" % (
                    len(sw_.spans), sum(1 for v in sw_.spans if v["load"]),
                    sum(((v["t1"] or time.time()) - v["t0"]) for v in sw_.spans)))
            if state.get("border_shadow"):
                run.say("Home session: the away border would have stopped it %d times (%d where home's border did "
                        "too): border_shadow.csv" % (len(state["border_shadow"]),
                                                     sum(1 for v in state["border_shadow"] if v[4])))
        except Exception as e:
            run.say("(processing summary failed: %s)" % e)
        if run.end_reason:
            state["session_end"] = dict(run.end_reason)
            run.say("Session ended: %s." % run.end_reason["why"])
        state.save()
        run.trackf.close()
        if sim is not None:
            stop_sim(sim, sdir)
            try:
                rep = dry_run_report(sdir)
                run.say("Dry-run check against the simulator's truth: " + json.dumps(rep["summary"]))
            except Exception as e:
                run.say("(dry-run report failed: %s)" % e)
        run.say("Saved in %s" % sdir)
    return code


class ShiftedFloor:
    """A camera whose floor positions are moved by (dx, dy) cm (--dry-guard-shift)."""

    def __init__(self, cam, dx, dy):
        self.cam, self.dx, self.dy = cam, dx, dy

    def to_floor(self, u, v, z=0.0):
        x, y = self.cam.to_floor(u, v, z)
        return x + self.dx, y + self.dy

    def __getattr__(self, k):
        return getattr(self.cam, k)


def dry_scene(spec):
    """--dry-scene: events injected during explore in a dry run (the 15:10 re-creation). Comma-separated
    ROBOT:KIND:N:WHAT[:AT_S]: during ROBOT's N-th explore leg of KIND (go, turn, ray = a 105 cm ray, bigturn = a turn
    of 150 deg or more), AT_S s (default 1.0) into it: WHAT 'gapS' (the camera loses it for S s), or a hold with
    Nick's answer 'c' / 'o'. Returns [dict]."""
    out = []
    for part in (spec or "").split(","):
        if not part.strip():
            continue
        f = part.strip().split(":")
        out.append(dict(robot=f[0], kind=f[1], n=int(f[2]), what=f[3], at=float(f[4]) if len(f) > 4 else 1.0,
                        done=False))
    return out


def slow_camera_hook(spec, run=None):
    """--dry-slow-camera PERIOD/ON/MS[/FIRST]: a camera hook that makes every picture MS ms slower for ON seconds of
    every PERIOD, from FIRST seconds after the start (the speed guard's test: a slow camera while robots drive)."""
    period, on, ms = (float(v) for v in spec.split("/")[:3])
    first = float(spec.split("/")[3]) if len(spec.split("/")) > 3 else 0.0
    t0 = time.time() + first
    state = dict(on=False)

    def hook(img, t):
        el = time.time() - t0
        slow = el >= 0 and (el % period) < on
        if slow != state["on"]:
            state["on"] = slow
            if run is not None:
                run.say("  (dry run: the camera %s)" % ("SLOWED: +%.0f ms a picture" % ms if slow else "back to speed"),
                        quiet=True)
                run.st.setdefault("dry_slow_camera", []).append([round(time.time(), 1), slow])
        if slow:
            time.sleep(ms / 1000.0)
    return hook


def start_sim(state, sdir):
    simdir = os.path.join(sdir, "sim")
    os.makedirs(simdir, exist_ok=True)
    for f in ("STOP", "sim_info.json", "BRICKS_ON"):
        if os.path.exists(os.path.join(simdir, f)):
            os.remove(os.path.join(simdir, f))
    sides = (state["setup"].get("args") or {}).get("board_sides") or "1,1"
    sim = subprocess.Popen([sys.executable, os.path.join(HERE, "e02_sim.py"), "--out", simdir,
                            "--xboard-side", sides.split(",")[0], "--yboard-side", sides.split(",")[1],
                            "--robots", ",".join(state["robots"]), "--seed", str(state["seed"] % 1000 + 1),
                            "--start", (state["setup"].get("args") or {}).get("dry_start") or "parks"]
                           + (["--bricks=" + (state["setup"].get("args") or {}).get("dry_bricks")]
                              if (state["setup"].get("args") or {}).get("dry_bricks") else [])
                           + (["--kept-pose"] if (state["setup"].get("args") or {}).get("dry_kept_pose") else [])
                           + (["--turn-fault", (state["setup"].get("args") or {}).get("dry_turn_fault")]
                              if (state["setup"].get("args") or {}).get("dry_turn_fault") else [])
                           + (["--fps", str((state["setup"].get("args") or {}).get("dry_camera_fps"))]
                              if (state["setup"].get("args") or {}).get("dry_camera_fps") else [])
                           + (["--drop-in-pct", str((state["setup"].get("args") or {}).get("dry_drop_pct"))]
                              if (state["setup"].get("args") or {}).get("dry_drop_pct") else [])
                           + (["--low-contrast", (state["setup"].get("args") or {}).get("dry_low_contrast")]
                              if (state["setup"].get("args") or {}).get("dry_low_contrast") else [])
                           + (["--drop-first=" + (state["setup"].get("args") or {}).get("dry_drop_first")]
                              if (state["setup"].get("args") or {}).get("dry_drop_first") else []),
                           stdout=open(os.path.join(simdir, "sim_stdout.txt"), "a"), stderr=subprocess.STDOUT)
    if (state["setup"].get("args") or {}).get("dry_bricks_now"):
        # --dry-bricks-now: the bricks stand there from the start (objects past an open edge, boards out from the
        # plywood: explore's creep test)
        open(os.path.join(simdir, "BRICKS_ON"), "w").close()
    if (state["setup"].get("args") or {}).get("no_boards"):
        # --no-boards: the simulator's boards go at once (both sides open edges, as on the real table tonight)
        for which in ("x", "y"):
            open(os.path.join(simdir, "REMOVE_BOARD_" + which), "w").close()
    rb = (state["setup"].get("args") or {}).get("dry_remove_board") or ""
    if rb:
        # the remove-a-board test in the simulator: the board goes after this many seconds, the runner not told
        which, _, after = rb.partition("@")
        threading.Timer(float(after or 600), lambda: open(os.path.join(simdir, "REMOVE_BOARD_" + which), "w").close()
                        ).start()
    t0 = time.time()
    while time.time() - t0 < 40 and not os.path.exists(os.path.join(simdir, "sim_info.json")):
        if sim.poll() is not None:
            raise SystemExit("The simulator stopped at once: see %s" % os.path.join(simdir, "sim_stdout.txt"))
        time.sleep(0.3)
    time.sleep(1.0)
    return sim


def stop_sim(sim, sdir):
    if sim is None:
        return
    open(os.path.join(sdir, "sim", "STOP"), "w").close()
    try:
        sim.wait(timeout=10)
    except subprocess.TimeoutExpired:
        sim.kill()


def dry_run_report(sdir):
    """Score a dry run against the simulator's truth: the camera tracker's error, the learned corrections
    against the robots' true errors, and how the runs went."""
    sim = os.path.join(sdir, "sim")
    truth = defaultdict(list)
    with open(os.path.join(sim, "truth.csv")) as f:
        for r in csv.DictReader(f):
            truth[r["robot"]].append((float(r["t"]), float(r["x_cm"]), float(r["y_cm"])))
    errs = []
    with open(os.path.join(sdir, "track.csv")) as f:
        rows = list(csv.DictReader(f))
    for robot, tr in truth.items():
        ts = np.array([t[0] for t in tr])
        xs = np.array([t[1] for t in tr])
        ys = np.array([t[2] for t in tr])
        for r in rows:
            if r["robot"] != robot:
                continue
            t = float(r["t"])
            if t < ts[0] or t > ts[-1]:
                continue
            errs.append(math.hypot(float(r["x_cm"]) - np.interp(t, ts, xs), float(r["y_cm"]) - np.interp(t, ts, ys)))
    info = json.load(open(os.path.join(sim, "sim_info.json")))
    state = json.load(open(os.path.join(sdir, "e02_state.json")))
    fits = {}
    for robot, tr in info["robots"].items():
        v = (state["fits"].get(robot) or {}).get("values")
        fits[robot] = dict(true=dict(gyro_scale=round(tr["g_true"], 4), gyro_drift_dps=round(tr["gyro_drift_dps"], 4),
                                     wheel_scale=round(tr["w_true"], 4)), learned=v)
    sched = state["schedule"]
    by = defaultdict(list)
    for r in sched:
        if r["status"] == "done" and r.get("end_err_cm") is not None:
            by[r["cond"]].append((r["end_err_cm"], r.get("far_err_cm")))
    e = np.array(errs) if errs else np.array([np.nan])
    summ = dict(track_samples=len(errs), track_err_median_cm=round(float(np.median(e)), 2),
                track_err_p95_cm=round(float(np.percentile(e, 95)), 2) if errs else None,
                runs_done=sum(1 for r in sched if r["status"] == "done"),
                runs_failed=sum(1 for r in sched if r["status"] == "failed"),
                runs_per_condition={c: len(by[c]) for c in by},
                end_err_mean_cm={c: round(float(np.mean([v[0] for v in by[c]])), 2) for c in by},
                end_err_sd_cm={c: round(float(np.std([v[0] for v in by[c]], ddof=1)), 2) if len(by[c]) > 1 else None
                               for c in by},
                recoveries={k: sum(1 for r in sched if r.get("recovery") == k)
                            for k in {r.get("recovery") for r in sched if r.get("recovery")}},
                watchdog_stops=len(state.get("watchdog_stops") or []),
                help_calls=len(state.get("help_calls") or []),
                find_failures=sum(1 for r in sched if "find_failed" in (r.get("flags") or [])),
                far_err_mean_cm={c: round(float(np.mean([v[1] for v in by[c] if v[1] is not None])), 2) for c in by},
                board_true=info["board"], board_used=state.get("board"))
    rep = dict(summary=summ, fits=fits)
    with open(os.path.join(sdir, "dry_run_report.json"), "w") as f:
        json.dump(rep, f, indent=1)
    return rep


if __name__ == "__main__":
    _code = main()
    # 2026-10-24 Pi, 20:43: after "Saved in ..." the runner never exited, so the plan never uploaded. The likely
    # cause: a console-answer thread still blocked in input() (web_ask starts one per question when a terminal is
    # attached) holds stdin's lock, and Python's shutdown waits on it. Everything is saved and closed by now
    # (main's finally): leave at once.
    try:
        sys.stdout.flush()
        sys.stderr.flush()
    finally:
        os._exit(int(_code or 0))
