#!/usr/bin/env python3
"""Cautious calibration run at the school table, zones 1-2 (Nick and Jay-Sun, 4 Nov 2026).

Nick (4 Nov): "Get enough data without calibration to use as a base level of measurement. But use calibration for
safety purposes like keeping it from the edge of the table and returning it to a parked position. After that, perform
a series of runs with updated calibration." Experiment speed (10 cm/s drive, 60 deg/s turns, 5 cm/s final approach) so
the calibration matches the experiments.

In order (every step logged in the session folder, logs/<date-time>-calib[-dry]/):
  0. find      the robot(s) on the WiFi (`status` to the given addresses and the broadcast; a sleeping robot is sent
               `wake` until it answers), the edge guard switched on, the robot's CURRENT corrections read with `get`
               and saved (robot_values_before.json) before anything changes.
  1. bind      each robot does the sync wiggle (experiments.py: a quarter turn and back) and the roof tag whose heading
               follows that wiggle is bound to it (binding.json). Refused, in plain words, when no tag moves or two do.
               Named from the WiFi name (AL-1DC0) unless the site names it. `--binding fixed`: the old way (the site's
               tags.ids), kept as a switch.
  A. baseline  with the robot's current values: in-place turns at the park point (both directions; also gives the
               tag's offset from the turning point), straight rolls of known lengths, a still period (gyro drift), then
               the P01-P20 tour. Every leg is driven on wheels + gyro alone (`turn` / `go` legs, the robot's own pose,
               never corrected by the camera); the camera measures where it actually stopped. The camera is used for
               SAFETY only: before a leg that would take the robot (from where the camera sees it) near an edge, a
               keep-out or the edge of the camera's view, the pose is corrected first (logged), and to park at P11.
  B. fit       wheel_scale, gyro_scale and gyro_drift_dps from phase A's legs (least squares, with +- values), `set`
               on the robot, read back with `get` (sent again once if it does not read back; the old values kept to
               undo: --undo-from).
  C. series    TOURS more tours of P01-P20 with the new values (in order, reversed, shuffled with a recorded seed), the
               same per-leg errors measured. Stops cleanly on low battery (engine_checks.battery_floor). Always ends
               parked at P11 and sends `sleep 10`.
Cautious: the roof tag lost for more than LOST_HOLD_S while driving -> `stop` and hold until it is seen again (up to
LOST_GIVEUP_S; then the run ends where it is). `--lost-policy keep_going` is the experiments' policy (drive on).

Writes: status.json (Mission Control's card), events.jsonl, legs.csv (every leg: commanded, robot's own, camera's),
camera_track.csv (every camera read), odometry_raw.csv (the robot's own pose, anchored once, never camera-corrected),
odometry_corrected.csv (the same with the safety corrections), fit.json, binding.json, report.md + charts.

    python3 tools/fleet/calib_run.py --robot 192.168.1.134                    # the school, on the Pi
    python3 tools/fleet/calib_run.py --dry                                    # the simulator dress rehearsal
    python3 tools/fleet/calib_run.py --dry --cam-z-cm 228.6                   # camera height read as above the table
"""
import argparse
import csv
import datetime as dt
import json
import math
import os
import random
import re
import signal
import socket
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, TOOLS, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "experiment")):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np   # noqa: E402

VERSION = "2026-11-12e19"   # e19 (the Fable audit H1): stop to every robot FIRST at every ending, an unexpected error too (stop_every_robot), then the saves. Before: "2026-11-12e18"   # e18 (Nick 12 Nov decision 2: never still in calibration too): a robot held because its roof tag is lost never just stands: its find-me wiggles come every derived pause (its tag's fix interval + 0.5 s, not doubling to a minute) and between them it creeps 2-4 cm toward the readable ground the map knows (a turn toward it and a short drive, only where its circle and error stay clear) (MIRLT_NEVER_STILL=0: the e16 doubling wait). Before: "2026-11-11e16"   # e16 (11 Nov, school 10 Nov 17:02: wheel_scale 0.73 +- 0.20 refused while the run's own drives said 0.988): a leg whose "started" report never came back (UDP) took its END pose as its start, so its own length read 0 cm against the camera's 44.5 cm; that one leg tilted the scale + coast fit. Such a leg is now marked (start_missing) and kept out of the fit and the pool, and the fit leaves out any drive whose own count missed half of what the camera saw (fit_values; the 17:02 legs now give 0.970 +- 0.004 with 0.85 cm coast). Before: "2026-11-11e15"   # e15 (school 10 Nov 18:54): heading_check_patient (a lost tag: tried again where it reads, standing and reading longer from its measured fix interval; then the remembered turn; then the finding stroke; else "unknown"); the calibration's heading check and heading watch remember each tag's turn (tag_heading_store); a lost tag no longer ends the calibration (measured again); MIRLT_HEADING_CHECK=once the old check; MIRLT_SIM_HC_LOSS the simulator's lost tag. Before: "2026-11-10e14d"   # e14d: a robot brought back is bound by its tag's first read at the motion detector's spot (rehearsal c7f: the pin check wiggle at a dim border ended the run). the "while parking" end fixed at its root: staging at the start parked with park("stage"), which set _ending for the whole run (school 16:22 and 17:02); now only the run's own end parks as ending, and it is cleared after. The pool weights each run's legs by that run's own noise and drops outlier legs (school 17:02: wheel_scale 0.73 +- 0.20). e14c (17:2x, Nick at school): a dark picture holds the robot (no legs, no recovery drives, no wiggles, no end, no sleep) until the camera has adjusted (dark_watch); --sim-blackout. e14b (10 Nov 16:40-17:00): calibration evidence pooled across runs (calib_pool: every measured leg of the robot on this site, finished or not, weighted by age and battery; a run starts from the pooled values, measures only what is still uncertain, saves once the pooled +- is no wider than one finished run's; MIRLT_CALIB_POOL=0 each run on its own); parking after the tour never gives up (school 16:22: MIRLT_PARK_GIVEUP=1 the e13 end); a recovery with no reliable spot heads for the readability map; the setup order said at the start. e14 (10 Nov): every robot found by its own small move at the start (motion_find: anywhere on the table, with or without a tag that reads; one found where its tag does not read drives itself back, never giving up; MIRLT_CAMERA_DISCOVERY=off the wiggle alone); calibration on any surface (its own spots inside the readable area when the site has no proven stickers: --spots / MIRLT_CALIB_SPOTS / calibration.spots); the readability evidence chosen and kept by light and tag (readmap_store; MIRLT_READMAP_STORE=0 the old readmap.json); a tour point whose tag was just lost goes to the end of the tour once, then is skipped (MIRLT_RETRY_LOST=again the old aim-again); the battery floor from the site / Mission Control said at the start; LOW-BATTERY CALIBRATION under 40 %; --sim-dark, --sim-extra-camera, --sim-light, --sim-start X,Y. VERSION_10I = "2026-11-10i"   # 11-10i: the binding looks over longer windows and waits for a fresh read after each wiggle half (a tag that reads only now and then, school 15:00). 11-10h: the never-give-up hold makes find-me wiggles (when its turning circle + error is clear) so the motion detector can find it and the recovery bring it back (Nick 14:32: motion, not hands). 11-10e13 (e13): never give up (the recovery drives blind while its bubble + stop cannot reach the edge margin, no 8 cm / 40 cm caps: MIRLT_BLIND_RULE=cap; a lost tag holds with no time limit, asks for a hand; cam_pose_sure and every unguarded camera pose wait the same way; MIRLT_CALIB_GIVEUP=1 the 30 s end), readability per tag, TURN DID NOT SETTLE, stop resent after the curve phase, the motion detector on in its bridge; the camera check before binding (camera_feed: no pictures is said as that, never the robot or tag); tracked for the sleep after a run.
VERSION_10G = "2026-11-10g"     # 11-10g: a standing robot is never given up for sparse tag reads: it is measured over longer windows (Nick 13:34; dim light). 11-10f: the start waits for the camera fix after the wiggle (dim light, ~7 fps: refused at once, school 13:25). 11-10e: a run ended for poor tag reads DOES try to park again (Nick 12:48: never give up on getting back to where it is seen); it drives only with a camera fix (cam_pose_sure waits for one). 11-10d: no crash parking with no camera fix (RunEnd, caught: "Could not park"). 11-10c: Phase A waits for the camera fix instead of crashing on None (school 12:35). 11-10b (e12, Nick 10 Nov): the curve phase (phase R: constant-curvature arcs left and right at free roam's radii and two speeds, fitted from the camera, arc_k_scale / arc_d_scale saved when they agree; --arc-phase auto/on/off), points in your order (--visit-order), the proven workspace for the tours' room, the camera self-check at the start. VERSION_10A was:
VERSION_10A = "2026-11-10a"     # 11-10a: a leftover camera reader holding port 5011 is stopped first (stale_bridge.py). 11-05i (9 Nov, school): a tripped stop mid-tour crashed the run (TypeError: ev() got two
                             # values for kind); the trip event now says stop_kind. VERSION_H was:
VERSION_H = "2026-11-05h"     # 11-05h: stop lead per speed (school 19:34: the scale fit was right; a fixed ~0.3 / 0.5 cm
                            # overrun per leg at 5 / 10 cm/s read as "+1 cm/m after the fit"). Shared observe checks.
VERSION_G = "2026-11-05g"     # 11-05g: --view-x-max (zone 1 only while the far end reads poorly).
VERSION_F = "2026-11-05f"     # 11-05f: heading check (the school tag reads 180 deg off the nose: legs ran backwards, 1 cm from the edge).
VERSION_E = "2026-11-05e"     # 11-05e: --safe-edge / calibration.safe_edge (school: 2, the tape seam tripped the line sensors).
VERSION_D = "2026-11-05d"     # 11-05d: recovery jump used the camera time as a position; hold again until a pose (school 18:33 crash). 11-05c: lost-tag recovery; arc phase
PY = sys.executable
LOG_PORT, CMD_PORT = 5005, 5006
FIX_PORT = 5013          # the tag bridge's (or the simulator's) camera reads come here (run_plan uses 5010)

# ------------------------------------------------------------------------------------------------ chosen numbers
SPEED_CMS = 10.0         # Nick: experiment speed (experiments.SPEED cruise). No DOE: matches the experiments
TURN_DPS = 60.0          # Nick: experiment turn speed. Set through turn_max_rpm (below). No DOE
FINAL_CMS = 5.0          # Nick: the final approach speed. No DOE
FINAL_CM = 5.0           # chosen: the last 5 cm of every leg at FINAL_CMS. DOE note: 3 / 5 / 10 cm against the leg's
                         # end error and time per point
WHEEL_R_CM, TRACK_CM = 1.7, 9.0    # the Alvik's nominal wheel radius and track (tools/sim/config.py)
TURN_RPM = round(math.radians(TURN_DPS) * TRACK_CM / 2.0 / (2 * math.pi * WHEEL_R_CM) * 60.0, 1)   # 26.5 rpm top wheel
                         # speed in a turn = 60 deg/s on nominal wheels. Derived, not measured: the report gives the
                         # turn rate the camera saw
DWELL_S = 3.0            # Nick / Jay-Sun: 3 s stop at each point
SETTLE_S = 0.4           # chosen: after a leg ends, wait this long, then average the camera over MEASURE_S
MEASURE_S = 1.2          # chosen: ~12 reads at 10 a second. DOE note: 0.6 / 1.2 / 2.4 s against the per-leg error's
                         # spread with the robot standing still (still period)
LOW_BATTERY_NOTE_PCT = 40   # (e14, the coordinator 10 Nov) a calibration started under this is marked a "low-battery
                         # calibration" in its report (values taken on a weak battery). Chosen; DOE row: calibration
                         # against battery (docs/doe-backlog.md)
LOST_HOLD_S = 1.0        # chosen (cautious): the tag unseen this long while driving -> stop and hold. At 10 cm/s that
                         # is 10 cm blind at most. DOE note: 0.5 / 1 / 2 s against holds per tour and how far it drives
                         # blind (the camera's dropout lengths from camera_track.csv)
STILL_MEASURE_MAX_S = 15.0  # chosen (11-10g): the longest window a standing robot's tag is measured over; DOE: against reads a second
LOST_GIVEUP_S = 30.0     # BEFORE e13 (now the switch MIRLT_CALIB_GIVEUP=1): held this long and still not seen -> the run
                         # ended where it was. e13 (Nick 10 Nov: robots never give up; the school calibration 20261110-121215
                         # ended this way after "go 63 P11"): a robot standing still does not get riskier with time, so the
                         # hold has no time limit: it stays still and safe, asks for a hand loudly, takes the motion
                         # detector's sightings (the tag bridge's motion source) and tries the recovery again from there
HOLD_SAY_S = 60.0        # (e13) the NEEDS A HAND line is said again this often while it holds: display only, no DOE
# (11-05c, Nick 5 Nov 16:30/16:34) lost-tag RECOVERY instead of stop-and-hold (the old hold: --lost-policy hold /
# site calibration.lost_policy "hold"). Straight legs with turns on the spot only (school runs never use curves):
# turn towards the nearest spot where the tag read reliably, drive there in short steps, each checked so the whole
# dead-reckoning bubble stays clear of the edges (+ the camera's view) and the outlets / tiles / parked robots
RECOVER_STEP_CM = 5.0    # chosen: one blind step (at FINAL_CMS); the check is redone before each. DOE note: 3 / 5 / 10
                         # cm against recoveries that end in a hold and the time to be seen again
RECOVER_MAX_CM = 40.0    # chosen: at most this far blind in a recovery (run_plan's BLIND_MAX_CM, Nick 29 Oct)
RECOVER_BUBBLE_CAP = 8.0  # BEFORE e13 (now the switch MIRLT_BLIND_RULE=cap): over this, stop and wiggle, then the hold.
BLIND_RULE = __import__("safety_group").value("MIRLT_BLIND_RULE")   # (e13, Nick 12:51 "never give up", CLAUDE.md) "edge": a robot
                         # driving blind goes on while its whole possible error (the bubble grown with every cm and degree
                         # blind + the body + its stop) cannot reach an edge along the step; no fixed bubble or distance
                         # cap. "cap": RECOVER_BUBBLE_CAP 8 cm and RECOVER_MAX_CM 40 cm as before
BLIND_BASE_CM = 2.0      # the bubble: MEASURED 29 Oct (run_plan BLIND_ERR_*): p95 2.0 cm + 0.07 cm per cm blind,
BLIND_PER_CM = 0.07      # calibrated robots (wheel_scale / gyro_scale fitted)
BLIND_PER_CM_UNCAL = BLIND_PER_CM + (0.10 - 0.04)   # uncalibrated (phase A from the defaults, staging): + the
                         # difference of the legs' safety allowances (SAFE_FRAC_UNCAL - SAFE_FRAC). Chosen, as those
BLIND_TURN_FRAC = 0.03   # heading error per degree turned blind (free roam's, chosen); uncalibrated 0.07 (6 deg on a
BLIND_TURN_FRAC_UNCAL = 0.07   # 90 deg turn: the e04 rehearsal's 173 deg turn 6 deg short). Chosen
RECOVER_WIGGLE_DEG = 15.0  # the last resort's find-me wiggle (+-), only with the turn circle and bubble clear
VIEW_X_MAX_CM = 78 * 2.54  # Jay-Sun: the one camera reads a roof tag anywhere in zones 1-2 (x < 78 in)
VIEW_MARGIN_CM = 6.0     # chosen: the robot's middle stays this far inside that line (half the body + 1 cm)
EDGE_AIM_CM = 1.0        # chosen: a point nearer an edge than edge_in_cm + this is aimed this much further in (P04:
                         # 13.3 cm from the edge is aimed at 14.0), so a leg's own error cannot take it under the margin
KEEPOUT_CLEAR_CM = 6.72 + 2.0   # the body's half diagonal + 2 cm from any keep-out or reference tile
SAFE_BUF_CM = 3.0        # chosen: a leg's predicted path (from where the camera sees the robot) must stay this much
                         # inside the edge margin + 4 % of the leg's length, else the pose is corrected first
SAFE_FRAC = 0.04         # a calibrated robot's leg: 4 % of its length as its possible error (as since 4 Nov)
SAFE_FRAC_UNCAL = 0.10   # (5 Nov 11:00, Nick: "cautious about the table edges") an uncalibrated robot's leg: 10 %
                         # (rehearsal r1: wheels 1.07, gyro 0.94 took AL-2B7F 9.9 cm from the edge with 4 %). Chosen:
                         # 7 % in length plus sin(6 deg) across. DOE note: 6 / 10 / 14 % against the closest approach to
                         # the edges and the points skipped in phase A, with true values 0.90-1.10
NEAR_EDGE_STAGE_CM = 10.0  # chosen: a point within edge_in + 8 cm of an edge: driven to a stop 10 cm short (odometry),
                         # then the last 10 cm from the camera's pose (safety)
TURNS_A = (90, 90, 90, 90, -90, -90, -90, -90, 170, -170, 170, -170)   # chosen: 12 in-place turns, 1400 deg, both
                         # directions (170, not 180: a half turn's direction is ambiguous, the robot takes the short way
                         # to its target heading). Gives gyro_scale to about +-0.002 (1 sigma) with 0.5 deg per measured turn.
                         # DOE note: 6 / 12 / 24 turns against the SE (fit.json)
ROLLS_A = (20.0, 45.0, 70.0, 20.0, 45.0, 70.0)   # chosen: out-and-back rolls (540 cm, 12 legs) from P11 towards -x
                         # (70 cm keeps 4 cm off the zone 1 outlet's clearance): with the tour's legs, wheel_scale and
                         # the coast per leg to about +-0.003 (1 sigma). DOE note: lengths and count against the SE
STILL_S = 60.0           # chosen: gyro drift from 60 s standing still (+-0.01-0.02 deg/s). DOE note: 30 / 60 / 120 s
STILL_C_S = 30.0         # chosen: a shorter still period at the start of each phase-C tour (drift after the fit)
TOURS_C = 3              # Nick: "a series of runs" (2-3): in order, reversed, shuffled
TOUR_SEED = 20261104     # chosen: the shuffled tour's seed (recorded)
ENOUGH = {"wheel_scale": 0.008, "gyro_scale": 0.005, "gyro_drift_dps": 0.03}   # "enough" phase-A data: each value's
                         # 95 % +- (1.96 x SE) under this before it is used. Chosen: 0.8 % in the wheels is 8 mm per
                         # metre (the coast per leg is fitted beside it, which costs precision), 0.5 % in the gyro is
                         # 0.45 deg per quarter turn, both well under the ~3 % expected to be removed; drift 0.03 deg/s
                         # is 1.8 deg a minute. DOE note: turns / rolls / still seconds against these
BOUNDS = {"wheel_scale": (0.90, 1.10), "gyro_scale": (0.90, 1.15), "gyro_drift_dps": (-2.0, 2.0)}
PARAM_TOL = 0.0006       # a read-back matches within this (the firmware prints 3 decimals; doe_loop.PARAM_TOL)
PARAMS = ("wheel_scale", "gyro_scale", "gyro_drift_dps")
LEADS_FILE = "stop_leads.json"   # (11-05h) per WiFi name: the stop lead per speed, kept on the Pi
OVERRUN_MIN_N = 5        # chosen: a speed's overrun is used only from this many phase-A legs (the real run had 17 at
                         # 5 cm/s and 21 at 10 cm/s, spread 0.1-0.2 cm). DOE note: not needed, a median of 5+ is stable
LEAD_MAX_CM = 1.5        # chosen: a stop lead larger than this is not believed (a sign of a bad camera read, not a
                         # slide), the leg is sent as asked
EDGE_GUARD = ["set safe_edge 3", "set edge_dir 0", "set edge_thr 150", "set edge_n 3", "set edge_bot_cm 11",
              "set bumper_cm 0"]       # mission_run.EDGE_GUARD (lost at every power-off)
LEG_TIMEOUT_S = 40.0
HEADING_CHECK_CM = 6.0   # (11-05f) the forward nudge that finds which way the tag reads against the nose. Chosen:
                         # several times the camera's ~0.3 cm noise, short enough to be clear both ways. No DOE
GO_PIECE_CM = 100.0      # a longer drive is made in equal pieces of at most this (the firmware refuses go legs over
                         # e02_go_max_cm, 110 by default). Each piece is measured as its own leg
MAX_TURN_DEG = 170.0     # a bigger turn is made as two halves (a half turn's direction is ambiguous)
BATTERY_CHECK_EVERY = 6  # chosen: `status` every 6 points
JAY_IN = {"P01": 2.4, "P05": 2.7, "P11": 4.5, "P15": 5.2}   # Jay-Sun 3-4 Nov: how far the robot came out, no height
                         # correction, always away from the camera
PARK_SPOTS = ("P11", "P01", "P09")   # (5 Nov, 11:00 Nick: P01 instead of P05) where each robot is calibrated and
                         # parked, in calibration order. P01: 30.5 cm from both edges, nearest the camera, rolls along +x.
                         # P09 (third, chosen): 21.5 cm from the y = W edge, in the A1 corner, off the tour's main legs,
                         # 29 cm or more from every other sticker but P05 (34) and P06 (36), rolls along +x at y 146.
                         # The site's calibration.park_spots replaces this list. Before 11:00 5 Nov (kept for the record):
PARK_SPOTS_E03 = ("P11", "P05", "P01")   # the
                         # first robot P11 (as on 4 Nov); the second P05 (chosen: the zone-1 corner sticker furthest from
                         # P11, 141 cm, 15.9 cm in from the x = 0 edge, near the camera, where few tour legs pass; its
                         # rolls go along +x in open table); a third P01 (corner 1). Each robot's own tours skip the
                         # others' spots. Chosen, not derived. DOE note: a simulator comparison of P01 / P03 / P05 / P09
                         # as the second spot against points skipped and detours per tour would decide it
PARKED_R_CM = 6.72 + 2.0  # (5 Nov) a robot standing still is kept out as a circle this big round where the camera sees it
                         # (its body's half diagonal + 2 cm for the camera and its turning); the moving robot's middle
                         # then stays KEEPOUT_CLEAR_CM further off (centres at least 17.4 cm apart). Chosen. DOE note:
                         # 1 / 2 / 4 cm extra against points skipped and the closest approach between the two robots
BASELINE_DEFAULTS = {"wheel_scale": 1.0, "gyro_scale": 1.0, "gyro_drift_dps": 0.0}   # (Nick, 5 Nov 11:00) the first
                         # calibration of a robot starts phase A from these (uncalibrated baseline), not saved values
PIN_CHECK_DEG = 45.0     # (5 Nov) a robot pinned to its roof tag: the check wiggle (a turn this big and back; only that
                         # tag must follow it). Chosen: 20x the camera's heading noise. DOE note: 20 / 45 / 90 deg against
                         # false failures on the real camera (it is the bind wiggle's own test, half the angle)
MIN_FIRMWARE = "2026-11-05a"   # (Nick, 5 Nov 11:00) the runs refuse a robot on older firmware; the site's
                         # calibration.min_firmware replaces it ("" or null = no check, the old behaviour)
PARK_TOL_CM = 2.0        # (5 Nov 11:00) parking: re-aimed from the camera while further than this from the spot (up to
PARK_TRIES = 2           # PARK_TRIES more legs). Chosen: the spot is where the next run starts and where the others keep
                         # out. DOE note: 1 / 2 / 4 cm against the extra legs and the next run's start error
AT_SPOT_CM = 5.0         # (5 Nov) a robot further than this from its spot is driven there (by the camera) first. Chosen
ROLL_DIRS = (180.0, 0.0, 90.0, -90.0)   # (5 Nov) phase A's rolls: the first heading with room for the longest roll
                         # (P11: -x as on 4 Nov; P05: +x)
READ_PX = 8.0            # simulator: a tag is read when its shorter side in the picture is at least this many pixels.
                         # Chosen so the simulated camera (lens 145 cm up) reads the roof tag in all of zones 1-2
                         # (9.5 px at x = 78 in) and not the far corner tags 21/22 (6.6 px), as Jay-Sun saw


# ------------------------------------------------------------------------------------------------ (11-10b) the curve phase
# Nick 10 Nov ("curve section in the calibration run"): robots that will drive curves (firmware 2026-11-05c or newer AND
# free roam's driving is curves) drive constant-curvature arcs (firmware follow step R, as free roam does) after phase B,
# on wheels + gyro alone (no camera fix sent during an arc), in the middle of the area the camera reads best, every arc
# checked first so its whole swept path, with its blind bubble, keeps the derived edge clearance (safety_rules); each
# arc fitted from the camera (freeroam_core.fit_arc_points, free roam's own maths) and arc_k_scale / arc_d_scale set and
# saved on the robot when the fits agree (freeroam_core.arc_agree: free roam's arc_save rule). Free roam's own in-run
# fitting stays as it is and refines these. Switch: --arc-phase auto (default) / on / off; site calibration.arc_phase;
# Mission Control Advanced "Curves in the calibration".
ARC_FW = "2026-11-05c"   # follow step R (freeroam_core.ARC_FW). Not chosen
ARC_RADII_CM = (12.0, 25.0, 50.0)   # free roam's menu (freeroam_core.K_BINS). Not chosen here
ARC_SWEEP_DEG = {12.0: 120.0, 25.0: 90.0, 50.0: 60.0}   # chosen: each arc's turn (25, 39, 52 cm of path): over the
                         # fit's 35 deg floor, short enough to fit beside the others in the middle. DOE: yes (backlog)
ARC_SPEEDS = (5.0, 10.0)  # chosen: the calibration's two speeds (final approach, cruise); the tight curve capped by its
                         # outer wheel (freeroam_core.v_cap: radius 12 at 9.0 cm/s). DOE: yes (backlog: 5 / 7.5 / 10 / 12)
ARC_ROUNDS = 2           # chosen: 12 arcs a round (3 radii x left/right x 2 speeds), 2 rounds = 24, so free roam's
                         # agreement rule (15 fits) holds with a few arcs lost. DOE: yes (backlog: how many are needed)
ARC_VERIFY = 3           # chosen: after saving, one arc per radius again to show the error is gone (not used in the fit)
ARC_FIT_MIN_K = 1 / 60.0  # chosen: the calibration also fits radius 50 (free roam fits radius 40 and tighter only)
ARC_K_ALLOW = 0.15       # chosen: each arc's room is checked as if it really turned 15 % tighter or wider and ran 15 %
                         # longer (the sim error is 6 %; free roam's fits stay in 0.6-1.6). DOE: no, a safety allowance
ARC_HEAD_STEP_DEG = 15.0  # chosen: start headings tried in this step, the smallest turn first. No DOE (a search step)
ARC_SAMPLE_CM = 2.0      # the room check's spacing along the arc. Chosen (under a quarter of the body). No DOE
ARC_CAM_DELAY_S = 0.0    # the camera reads are matched to the arc by their own picture time ("t"); this is only the
                         # fallback shift for reads without it. Not chosen (no fallback needed in the sim or tag bridge)


def loopback_up():
    """In a fresh network namespace (unshare -rn) the loopback starts down (sweeps/rp_shim.py's way)."""
    import fcntl
    import struct
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        ifr = struct.pack("16sH14s", b"lo", 0, b"\0" * 14)
        flags = struct.unpack("16sH14s", fcntl.ioctl(s, 0x8913, ifr))[1]
        if not flags & 1:
            fcntl.ioctl(s, 0x8914, struct.pack("16sH14s", b"lo", flags | 1, b"\0" * 14))
    except OSError:
        pass
    finally:
        s.close()


def say(s):
    print(time.strftime("%H:%M:%S ") + s, flush=True)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


OWN_SPOTS_N = 20         # (e14) the calibration's own spots: as many as the school's stickers (P01-P20). Not chosen here
OWN_SPOTS_MIN_CM = 25.0  # CHOSEN (e14): spots at least this far apart, so every tour leg is a measurable length (the
                         # rolls' shortest is 20 cm). DOE row (e14): 15 / 25 / 40 cm against the fit's +- per tour minute
OWN_SPOTS_GRID_CM = 5.0  # the candidates' grid (a search step, under the robot's half body). Not tuned; no DOE
OWN_SPOTS_KEEPOUT_EXTRA = 5.0   # CHOSEN (e14): spots this much further from a keep-out than the legs' own clearance,
                         # so the point's leg error does not bring it inside. No DOE (a margin over a margin)
EDGE_STAGE_ROOM = 8.0    # a spot this much further in than the edge margin needs no near-edge staging (Site.near_edge)


# =============================================================================================== the site
class Site:
    """The school site's table, points, keep-outs (outlets and reference tiles), edge margin and the camera's view."""

    def __init__(self, site, park_label="P11", spots=None):
        import sticker_points as SP
        import production as PR
        self.site = site
        t = site.get("table") or {}
        self.L, self.W = float(t.get("length_cm", 396.2)), float(t.get("width_cm", 167.6))
        # (e14, Nick 15:23: "if I clicked calibration on a new surface, it wouldn't know what to do?") the typed
        # stickers only where they are present and proven; otherwise the run makes its own spots (make_own_spots)
        self.spots_mode, self.spots_why = spots_mode(site, spots)
        if self.spots_mode == "stickers":
            pts, zones, _w = SP.points_cm(site)
        else:
            pts, zones = {}, {}
        self.points = {k: tuple(v) for k, v in pts.items()}
        self.zones = zones
        self.edge = float(site.get("edge_in_cm", 13.0))
        self.keepouts = [dict(k, kind="outlet") for k in PR.site_keepouts(site)]
        try:
            import ref_tags as RT
            rc = site.get("ref_tags") or {}
            if rc.get("use", True):
                cfg = dict(RT.default_config(), **rc)
                for k in RT.keepouts(RT.world_poses(cfg, self.L, self.W), cfg):
                    self.keepouts.append(dict(k, kind="tile"))
        except Exception as e:  # noqa: BLE001
            say("(reference tiles not added as keep-outs: %s)" % e)
        self.view_x = VIEW_X_MAX_CM - VIEW_MARGIN_CM
        hp = (site.get("homes") or {})
        self.park_label = park_label
        self.park_face = float(hp.get("face_deg", 90.0))

    def make_own_spots(self, readable, n_max=OWN_SPOTS_N):
        """(e14) The calibration's own spots, when the site has no proven stickers: inside the table the cameras
        measured, where the readability map says tags read (readable(x, y) -> bool), clear of the edges (the edge
        margin + EDGE_STAGE_ROOM, so no point needs the near-edge staging) and of every keep-out it knows (reference
        tiles, outlets, drops the proven workspace found); spread out (each new spot the candidate furthest from the
        ones taken, so the legs between them vary in length and heading), at least OWN_SPOTS_MIN_CM apart. Labelled
        S01, S02 ... in a serpentine order along the table (the tours' order). Returns words."""
        m = self.edge + EDGE_STAGE_ROOM
        g = OWN_SPOTS_GRID_CM
        cand = []
        x = m
        while x <= self.L - m + 1e-6:
            y = m
            while y <= self.W - m + 1e-6:
                if self.edge_dist((x, y)) >= m and self.keepout_dist((x, y))[0] >= KEEPOUT_CLEAR_CM + OWN_SPOTS_KEEPOUT_EXTRA \
                        and readable(x, y):
                    cand.append((round(x, 1), round(y, 1)))
                y += g
            x += g
        if not cand:
            self.points, self.zones = {}, {}
            return "no spot where the camera reads tags, clear of the edges and keep-outs, was found"
        cx = sum(c[0] for c in cand) / len(cand)
        cy = sum(c[1] for c in cand) / len(cand)
        taken = [min(cand, key=lambda c: math.hypot(c[0] - cx, c[1] - cy))]
        while len(taken) < n_max:
            best = max(cand, key=lambda c: min(math.hypot(c[0] - t_[0], c[1] - t_[1]) for t_ in taken))
            if min(math.hypot(best[0] - t_[0], best[1] - t_[1]) for t_ in taken) < OWN_SPOTS_MIN_CM:
                break
            taken.append(best)
        centre = taken[0]
        # the tours' order: a serpentine along the table (columns of OWN_SPOTS_MIN_CM)
        order = sorted(taken, key=lambda c: (int(c[0] // OWN_SPOTS_MIN_CM),
                                             c[1] if int(c[0] // OWN_SPOTS_MIN_CM) % 2 == 0 else -c[1]))
        self.points = {"S%02d" % (k + 1): c for k, c in enumerate(order)}
        self.zones = {k: 1 for k in self.points}
        self.own_centre = next(k for k, c in self.points.items() if c == centre)
        return ("%d spots of its own (no proven stickers: %s), %.0f cm or more apart, at least %.0f cm from the edges, "
                "where the camera reads tags (%d candidates on a %.0f cm grid)" % (
                    len(self.points), self.spots_why, OWN_SPOTS_MIN_CM, m, len(cand), g))

    def own_park_spots(self, n=3):
        """(e14) Park spots among its own spots: the one nearest the middle of the readable area first (rolls there have
        the most room), then the ones furthest from the spots already taken."""
        if not self.points:
            return ()
        out = [getattr(self, "own_centre", None) or next(iter(self.points))]
        while len(out) < min(n, len(self.points)):
            out.append(max((k for k in self.points if k not in out), key=lambda k: min(
                math.hypot(self.points[k][0] - self.points[o][0], self.points[k][1] - self.points[o][1]) for o in out)))
        return tuple(out)

    def set_parked(self, poses):
        """(5 Nov) the other robots standing still: {name: (x, y)} where the camera sees them, each kept out as a
        circle of PARKED_R_CM (replaces the previous set)."""
        self.keepouts = [k for k in self.keepouts if k.get("kind") != "robot"]
        for n, (x, y) in poses.items():
            poly = [(x + PARKED_R_CM * math.cos(math.radians(a)), y + PARKED_R_CM * math.sin(math.radians(a)))
                    for a in range(0, 360, 30)]
            self.keepouts.append({"name": "parked robot %s" % n, "polygon": poly, "kind": "robot"})

    def parked_near(self, p):
        """(name, distance) of a parked robot's keep-out nearer p than the clearance, else None."""
        for k in self.keepouts:
            if k.get("kind") == "robot":
                d = poly_dist(p, k["polygon"])
                if d < KEEPOUT_CLEAR_CM:
                    return k["name"], d
        return None

    def zone12(self):
        return sorted([k for k, z in self.zones.items() if z in (1, 2)], key=lambda s: int(s[1:]))

    def aim(self, lab):
        """Where a point is aimed: EDGE_AIM_CM further in when it lies within edge + EDGE_AIM_CM of an edge."""
        x, y = self.points[lab]
        m = self.edge + EDGE_AIM_CM
        return (min(max(x, m), self.L - m), min(max(y, m), self.W - m))

    def edge_dist(self, p):
        x, y = p
        d = min(x, y, self.L - x, self.W - y)
        ws = getattr(self, "ws", None)
        if ws is not None:
            # (11-10b, e12) the proven workspace: drops and unproven ground found by earlier runs here count as edges
            d = min(d, ws.edge_dist(x, y))
        return d

    def use_workspace(self, site, folder=None):
        """(11-10b, e12) The site's proven workspace (proven_workspace.py, saved by free roam and discovery): the typed
        table seen as a whole, plus the felt cells and drops earlier runs saved (used only on the same outline). Words."""
        import proven_workspace as PW
        ws = PW.ProvenWorkspace((0.0, 0.0, self.L, self.W))
        ws.see()
        p = PW.site_path(folder or (site.get("conf_dir") or "~/.config/mirlt"), site.get("name") or "school-conference")
        words = ws.merge_saved(p)
        if "added" in words and not words.endswith("0 felt cells and 0 drop cells"):
            self.ws = ws
        return words

    def keepout_dist(self, p):
        best = (1e9, "")
        for k in self.keepouts:
            d = poly_dist(p, k["polygon"])
            if d < best[0]:
                best = (d, k["name"])
        return best

    def near_edge(self, lab):
        return self.edge_dist(self.points[lab]) < self.edge + 8.0

    def via(self, a, b):
        """One waypoint round a keep-out in the way of a -> b (None if no single waypoint works): the corners of each
        keep-out pushed out by its clearance + 3 cm, the shortest clear one."""
        best = None
        for k in self.keepouts:
            P = np.array(k["polygon"], float)
            c = P.mean(0)
            for q in P:
                v = q - c
                w = q + v / (np.linalg.norm(v) or 1) * (KEEPOUT_CLEAR_CM + 3.0) * 1.42
                w = (float(w[0]), float(w[1]))
                if self.problems(a, w) or self.problems(w, b):
                    continue
                d = math.hypot(w[0] - a[0], w[1] - a[1]) + math.hypot(b[0] - w[0], b[1] - w[1])
                if best is None or d < best[0]:
                    best = (d, w)
        return best[1] if best else None

    def problems(self, a, b, buf=0.0, edge_buf=None):
        """Why the straight line a -> b is not safe ([] = safe): the robot's middle under the edge margin (less 0.5 cm
        of measuring), near a keep-out or a reference tile, or past the camera's view; buf: the leg's own possible
        error (cm) added to each. (5 Nov 11:00) edge_buf: a different allowance for the table's edges and the camera's
        view (an uncalibrated robot's larger one: an outlet plate is a keep-out, the edge is a fall)."""
        out = []
        eb = buf if edge_buf is None else edge_buf
        for k in range(1, 11):
            p = (a[0] + (b[0] - a[0]) * k / 10.0, a[1] + (b[1] - a[1]) * k / 10.0)
            e = self.edge_dist(p)
            if e - eb < self.edge - 0.5:
                out.append("the edge margin (%.1f cm from the edge, %.1f with the leg's possible error)" % (e, e - eb))
            d, n = self.keepout_dist(p)
            if d - buf < KEEPOUT_CLEAR_CM - 0.5:
                out.append("%s (%.1f cm)" % (n, d))
            if p[0] + eb > self.view_x:
                out.append("the edge of the camera's view (x %.0f cm)" % p[0])
            if out:
                break
        return out


def spots_mode(site, override=None):
    """(e14) ("stickers" | "own", why). The calibration's spots: the typed stickers when they are present AND proven
    (a points file of stickers, not the grid site_detect makes for a new area, every point on the table), else its own
    (make_own_spots). Switch: --spots, else MIRLT_CALIB_SPOTS, else the site's calibration.spots: auto (default) /
    stickers / own."""
    import sticker_points as SP
    v = (override or os.environ.get("MIRLT_CALIB_SPOTS") or ((site or {}).get("calibration") or {}).get("spots")
         or "auto").lower()
    pt = (site or {}).get("points") or {}
    pf = SP.points_file(site)
    if v == "own":
        return "own", "asked for (spots: own)"
    if not pf or not os.path.exists(pf):
        return "own", "the site has no points file"
    if "not stickers" in str(pt.get("_note", "")) or pt.get("kind") == "grid":
        return "own", "the site's points are a grid made for a new area, not stickers"
    try:
        pts, _z, _w = SP.points_cm(site)
    except Exception as e:  # noqa: BLE001
        return "own", "the points file could not be read (%s)" % e
    t = (site or {}).get("table") or {}
    L, W = float(t.get("length_cm") or 0), float(t.get("width_cm") or 0)
    off = [k for k, (x, y) in pts.items() if L and W and not (0 < x < L and 0 < y < W)]
    if not pts:
        return "own", "the points file is empty"
    if off and v != "stickers":
        return "own", "the stickers %s lie off the table measured (%.0f x %.0f cm): not proven" % (", ".join(off), L, W)
    return "stickers", ("the typed stickers (%d, all on the table)" % len(pts)) if v == "auto" else "asked for (spots: stickers)"


def solid(why):
    """A problem a detour can go round: an outlet, a reference tile, or (5 Nov) a parked robot."""
    return "outlet" in why or "reference" in why or "parked robot" in why


def compose(a, rel):
    """Pose a (x, y, th deg) moved by rel (dx, dy in a's frame, dth)."""
    c, s_ = math.cos(math.radians(a[2])), math.sin(math.radians(a[2]))
    return (a[0] + c * rel[0] - s_ * rel[1], a[1] + s_ * rel[0] + c * rel[1], wrap180(a[2] + rel[2]))


def relative(b0, b1):
    """The robot's own motion over a leg, in the frame of its pose at the leg's start (its LEG records). A robot with no
    camera fix starts every idle leg at (0, 0, 0) (firmware pose::start(false)); this works either way."""
    c, s_ = math.cos(math.radians(b0[2])), math.sin(math.radians(b0[2]))
    dx, dy = b1[0] - b0[0], b1[1] - b0[1]
    return (c * dx + s_ * dy, -s_ * dx + c * dy, wrap180(b1[2] - b0[2]))


def poly_dist(p, poly):
    """Distance from p to a polygon (0 inside)."""
    x, y = p
    inside = False
    n = len(poly)
    best = 1e9
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / ((y2 - y1) or 1e-12) + x1:
            inside = not inside
        dx, dy = x2 - x1, y2 - y1
        t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / ((dx * dx + dy * dy) or 1e-12)))
        best = min(best, math.hypot(x - x1 - t * dx, y - y1 - t * dy))
    return 0.0 if inside else best


# =============================================================================================== the robots' WiFi
class Link:
    """UDP to the robots (alvik_console's ports): commands to <ip>:5006, every line back on :5005 ("name|seq|line")."""

    def __init__(self, folder, bind_host=""):
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.rx.bind((bind_host, LOG_PORT))       # no SO_REUSEADDR: another program on the port is found here
        except OSError:
            raise SystemExit("STOPPED: port %d is in use: another program is listening to the robots (only one can at "
                             "a time): Mission Control's own run, the console, robot_sleep.py, or at school Jay-Sun's "
                             "own console. Close it or ask him to, then start again. Nothing moved." % LOG_PORT)
        self.rx.settimeout(0.2)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.lines = []          # (t, name, ip, text)
        self.ips = {}            # name -> ip
        self.cond = threading.Condition()
        self.stop_ = threading.Event()
        self.f = open(os.path.join(folder, "messages.csv"), "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "robot", "ip", "line"])
        threading.Thread(target=self._rx, daemon=True).start()

    def _rx(self):
        while not self.stop_.is_set():
            try:
                data, (ip, _) = self.rx.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            txt = data.decode("utf-8", "replace").strip()
            name, _, rest = txt.partition("|")
            _seq, _, line = rest.partition("|")
            t = time.time()
            if line.startswith("D,RAW,"):
                self._raw(t, name, line)      # (11-05h) the standing-tilt watch; RAW rows are not kept (50 a second)
                continue
            with self.cond:
                self.lines.append((t, name, ip, line))
                self.ips[name] = ip
                self.w.writerow(["%.3f" % t, name, ip, line])
                self.cond.notify_all()

    def _raw(self, t, name, line):
        """(11-05h, protection 2) roll / pitch / speed from a RAW row into this robot's observe.TiltWatch; a standing
        tilt over the limit is kept in self.tilt_hazard[name] for the run to act on."""
        try:
            if not getattr(self, "raw_cols", None):
                import alvik_console as AC
                self.raw_cols = list(AC.KNOWN_HEADERS.get("RAW") or [])
            row = dict(zip(self.raw_cols, line.split(",")[2:]))
            import observe as OB
            w = self.__dict__.setdefault("twatch", {}).setdefault(name, OB.TiltWatch())
            ev = w.feed(t, float(row["roll_deg"]), float(row["pitch_deg"]), float(row.get("v_cms") or 0.0))
            if ev:
                self.__dict__.setdefault("tilt_hazard", {})[name] = dict(ev, t=t)
        except (KeyError, ValueError, IndexError):
            pass

    def send(self, ip, cmd):
        try:
            self.tx.sendto(cmd.encode(), (ip, CMD_PORT))
        except OSError:
            pass

    def mark(self):
        with self.cond:
            return len(self.lines)

    def wait_for(self, pred, since, timeout):
        """The first line from index `since` on for which pred(name, line) is truthy, or None after timeout."""
        end = time.time() + timeout
        k = since
        with self.cond:
            while True:
                while k < len(self.lines):
                    t, n, ip, line = self.lines[k]
                    k += 1
                    r = pred(n, line)
                    if r:
                        return r
                left = end - time.time()
                if left <= 0:
                    return None
                self.cond.wait(min(left, 0.2))

    def close(self):
        self.stop_.set()
        time.sleep(0.3)               # the receiving thread sees stop (its socket waits 0.2 s at most)
        for so in ("rx", "tx"):
            try:
                getattr(self, so).close()   # (5 Nov) the port is free at once: mission_run's run_plan binds it next
            except (OSError, AttributeError):
                pass
        try:
            self.f.close()
        except OSError:
            pass


class Robot:
    """One robot through the Link: status, get / set with read-back, idle `turn` / `go` legs and their LEG records."""

    def __init__(self, link, name, ip):
        self.link, self.name, self.ip = link, name, ip
        self.leg_id = int(time.time()) % 100000 * 10 + 1000000   # above any id this robot heard before today
        self.cmds = None

    def ask(self, cmd, pred, timeout=3.0, tries=3):
        for _ in range(tries):
            k = self.link.mark()
            self.link.send(self.ip, cmd)
            r = self.link.wait_for(lambda n, l: n == self.name and pred(l), k, timeout)
            if r:
                return r
        return None

    def status(self):
        import engine_checks as EC
        r = self.ask("status", lambda l: EC.parse_status(l), 3.0)
        return r

    def get(self, param):
        rx = re.compile(r"^M,%s = (-?[0-9.]+)" % re.escape(param))

        def p(line):
            m = rx.match(line)
            return (float(m.group(1)),) if m else None     # a tuple: 0.000 is an answer too
        r = self.ask("get %s" % param, p, 2.5)
        return r[0] if r else None

    def set(self, param, value):
        self.link.send(self.ip, "set %s %.4f" % (param, value))
        time.sleep(0.15)

    def set_checked(self, param, value):
        """`set`, then `get`; sent again once if it does not read back. (ok, read value)."""
        got = None
        for _ in range(2):
            self.set(param, value)
            got = self.get(param)
            if got is not None and abs(got - value) <= PARAM_TOL:
                return True, got
        return False, got

    def leg(self, kind, value, timeout=LEG_TIMEOUT_S, on_tick=None):
        """An idle leg (`turn <deg> <id>` / `go <cm> <id>`): resent until queued, then waited for. Returns dict(result,
        info, start (x, y, th, ms), end (x, y, th, ms))."""
        self.leg_id += 1
        lid = self.leg_id
        cmd = "%s %.2f %d" % ("turn" if kind == "turn" else "go", value, lid)
        out = {"result": None, "info": "", "start": None, "end": None, "id": lid}
        rx = re.compile(r"^D,LEG,[^,]*,(\d+),(\w+),(-?[0-9.]+),(\w+),(\d+),(-?[0-9.]+),(-?[0-9.]+),(-?[0-9.]+),?(.*)$")
        k = self.link.mark()
        t_end = time.time() + timeout
        last_send = 0
        queued = False
        while time.time() < t_end:
            if not queued and time.time() - last_send > 1.0:
                self.link.send(self.ip, cmd)
                last_send = time.time()
            r = self.link.wait_for(lambda n, l: n == self.name and rx.match(l), k, 0.25)
            if on_tick and on_tick():
                self.link.send(self.ip, "stop")
            if not r:
                continue
            with self.link.cond:
                k = self._index_after(r, k)
            if int(r.group(1)) != lid:
                continue
            res = r.group(4)
            pose = (float(r.group(6)), float(r.group(7)), float(r.group(8)), int(r.group(5)))
            if res in ("queued", "dup"):
                queued = True
            elif res == "started":
                queued = True
                out["start"] = pose
            elif res in ("done", "failed", "refused"):
                out.update(result=res, info=r.group(9), end=pose)
                if out["start"] is None:
                    # (e16, school 17:02) the "started" report was lost: the end pose stands in for the start (as before,
                    # for the odometry paths), and the leg is marked so it is never taken as a measurement of the wheels
                    # or the gyro (its own length / turn would read 0)
                    out["start"] = pose
                    out["start_missing"] = True
                return out
        out["result"] = "timeout"
        out["diag"] = self.leg_diag(k, lid)        # (e13) what did come back: FOLLOW rows, LEG records, its pose
        return out

    def leg_diag(self, k0, lid):
        """(e13) After a leg timed out: what the robot did send meanwhile. A robot still FOLLOWING (its FOLLOW rows
        coming, no LEG record for this leg) is said as that, not as a plain timeout."""
        rows = [ln for ln in list(self.link.lines[k0:]) if ln[1] == self.name]
        fol = [ln[3] for ln in rows if ln[3].startswith("D,FOLLOW")]
        legs = [ln[3] for ln in rows if ln[3].startswith("D,LEG")]
        mine = [l_ for l_ in legs if (",%d," % lid) in l_]
        th = None
        for l_ in reversed(fol):
            try:
                th = float(l_.split(",")[11])            # FOLLOW: ..., x, y, th, ... (follow.h)
                break
            except (IndexError, ValueError):
                continue
        words = []
        if fol and not mine:
            words.append("the robot was still FOLLOWING (%d FOLLOW rows during the leg, no LEG record for leg %d): the "
                         "idle leg was not run as a leg" % (len(fol), lid))
        elif not mine:
            words.append("no LEG record for leg %d reached the calibration (%d lines from the robot meanwhile)" % (
                lid, len(rows)))
        else:
            words.append("leg %d was %s but never ended" % (lid, mine[-1].split(",")[4] if len(mine[-1].split(",")) > 4
                                                            else "seen"))
        return {"follow_rows": len(fol), "leg_records": len(mine), "lines": len(rows), "own_th": th,
                "words": "; ".join(words)}

    def _index_after(self, match, k):
        for i in range(k, len(self.link.lines)):
            if self.link.lines[i][3] == match.string:
                return i + 1
        return len(self.link.lines)


# =============================================================================================== the camera
class Eye:
    """The newest camera reads per tag id: {id: (t, x, y, th)} (the turning point's frame is applied by the run)."""

    def __init__(self, folder, port=FIX_PORT):
        self.loose = {}                    # (e13) tag id -> (t, x, y, src): motion / lock-on sightings
        self.latest = {}
        self.hist = {}
        self.lock = threading.Lock()
        self.stop_ = threading.Event()
        self.folder = folder               # (e13) the bridge's frames.json is in folder/bridge
        self.f = open(os.path.join(folder, "camera_track.csv"), "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "tag_id", "x_cm", "y_cm", "th_deg", "sigma_cm", "source"])
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.bind(("127.0.0.1", port))
        self.rx.settimeout(0.2)
        threading.Thread(target=self._rx, daemon=True).start()

    def tag_of(self, msg):
        r = str(msg.get("robot", ""))
        return int(r[3:]) if r.startswith("tag") and r[3:].isdigit() else None

    def transform(self, msg):
        return msg

    def _rx(self):
        while not self.stop_.is_set():
            try:
                data, _ = self.rx.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                msg = json.loads(data.decode())
            except ValueError:
                continue
            if msg.get("type") == "camera":
                # (11-10b, e12) the bridge's frame watch: no new picture (or pictures again), said loudly; a leg loses
                # the tag after LOST_HOLD_S and holds as before
                say(str(msg.get("words") or "CAMERA %s: %s" % (msg.get("cam"), msg.get("state"))))
                self.camera_state = (msg.get("state"), time.time())
                if msg.get("state") == "dark":
                    self.dark_t = time.time()            # (e14) the picture went dark: a camera problem
                continue
            if msg.get("type") != "fix":
                continue
            msg = self.transform(msg)
            if not msg:
                continue
            tid = self.tag_of(msg)
            if tid is None:
                continue
            t = time.time()
            if msg.get("th") is None or str(msg.get("src") or "").split(":")[-1] in ("motion", "lock", "ml"):
                # (e13) a motion / lock-on sighting (position only): kept apart, never taken as a tag read
                try:
                    with self.lock:
                        self.loose[tid] = (t, float(msg["x"]), float(msg["y"]), str(msg.get("src") or ""))
                except (KeyError, TypeError, ValueError, AttributeError):
                    pass
                continue
            th = float(msg.get("th", 0.0)) - (getattr(self, "sim_yaw_by", {}) or {}).get(tid, getattr(self, "sim_yaw", 0.0)) \
                + getattr(self, "yaw_off", {}).get(tid, 0.0)   # (11-05f) heading check
            rec = (t, float(msg["x"]), float(msg["y"]), ((th + 180.0) % 360.0) - 180.0)
            try:
                ft = float(msg.get("t")) if msg.get("t") is not None else t - ARC_CAM_DELAY_S
            except (TypeError, ValueError):
                ft = t - ARC_CAM_DELAY_S
            with self.lock:
                fh = self.__dict__.setdefault("fhist", {}).setdefault(tid, [])   # (11-10b) by the picture's own time
                fh.append((ft,) + rec[1:])
                if len(fh) > 4000:
                    del fh[:1000]
                self.latest[tid] = rec
                self.hist.setdefault(tid, []).append(rec)
                if len(self.hist[tid]) > 4000:
                    del self.hist[tid][:1000]
                self.w.writerow(["%.3f" % t, tid, "%.2f" % rec[1], "%.2f" % rec[2], "%.2f" % rec[3],
                                 "%.2f" % float(msg.get("sigma", 0) or 0), msg.get("source", "camera")])
                self.nrows = getattr(self, "nrows", 0) + 1
                if self.nrows % 50 == 0:
                    self.f.flush()

    def is_dark(self):
        """(e14) The bridge says the picture is dark (the camera adjusting to the light)."""
        st = getattr(self, "camera_state", None)
        return bool(st and st[0] == "dark")

    def loose_last(self, tid):
        """(e13) The newest motion / lock-on sighting of this tag's robot (t, x, y, src) or None."""
        with self.lock:
            return getattr(self, "loose", {}).get(tid)

    def seen_ids(self, within_s=1.5):
        now = time.time()
        with self.lock:
            return sorted(i for i, r in self.latest.items() if now - r[0] <= within_s)

    def last(self, tid):
        with self.lock:
            return self.latest.get(tid)

    def age(self, tid):
        r = self.last(tid)
        return 1e9 if r is None else time.time() - r[0]

    def window(self, tid, t0, t1):
        with self.lock:
            return [r for r in self.hist.get(tid, []) if t0 <= r[0] <= t1]

    def frame_window(self, tid, t0, t1):
        """(11-10b) The reads whose PICTURES were taken between t0 and t1 (host clock): (t, x, y, th)."""
        with self.lock:
            return [r for r in self.__dict__.get("fhist", {}).get(tid, []) if t0 <= r[0] <= t1]

    def measure(self, tid, dur=MEASURE_S):
        """Average pose over the next dur s (circular mean heading). None if fewer than 3 reads."""
        t0 = time.time()
        time.sleep(dur)
        rs = self.window(tid, t0, time.time())
        if len(rs) < 3:
            return None
        xs, ys = np.median([r[1] for r in rs]), np.median([r[2] for r in rs])
        th = math.degrees(math.atan2(np.mean([math.sin(math.radians(r[3])) for r in rs]),
                                     np.mean([math.cos(math.radians(r[3])) for r in rs])))
        return (float(xs), float(ys), float(th), len(rs))

    def close(self):
        self.stop_.set()
        time.sleep(0.3)               # the receiving thread sees stop (its socket waits 0.2 s at most)
        for so in ("rx", "tx"):
            try:
                getattr(self, so).close()   # (5 Nov) the port is free at once: mission_run's run_plan binds it next
            except (OSError, AttributeError):
                pass
        try:
            self.f.close()
        except OSError:
            pass


class SchoolCamera:
    """The school's one camera as a pinhole (simulator and the four-point check): lens at (x, y, z) cm in the table frame
    (z above the table top), looking down the table's length. C920e: 1280 x 960 (the site's capture), ~70 deg across."""

    def __init__(self, z_cm=145.0, behind_cm=45 * 2.54, y_cm=167.6 / 2, size=(1280, 960), hfov_deg=70.4,
                 look_at=(150.0, 83.8, 0.0)):
        import tag_cam as TCAM
        self.C = np.array([-behind_cm, y_cm, z_cm], float)
        self.R = TCAM.look_rotation(self.C, look_at)
        self.W, self.H = size
        self.f = (self.W / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)

    def project(self, P):
        Pc = (np.asarray(P, float) - self.C) @ self.R.T
        if np.any(Pc[:, 2] <= 1):
            return None
        u = self.f * Pc[:, 0] / Pc[:, 2] + self.W / 2.0
        v = self.f * Pc[:, 1] / Pc[:, 2] + self.H / 2.0
        return np.stack([u, v], 1)

    def tag_px(self, x, y, z, s, th_deg=0.0):
        """The tag's shorter side in pixels (None outside the picture)."""
        c, sn = math.cos(math.radians(th_deg)), math.sin(math.radians(th_deg))
        h = s / 2.0
        P = [[x + c * a - sn * b, y + sn * a + c * b, z] for a, b in ((-h, -h), (h, -h), (h, h), (-h, h))]
        uv = self.project(P)
        if uv is None or np.any(uv[:, 0] < 2) or np.any(uv[:, 0] > self.W - 2) or np.any(uv[:, 1] < 2) or \
                np.any(uv[:, 1] > self.H - 2):
            return None
        sides = [np.linalg.norm(uv[i] - uv[(i + 1) % 4]) for i in range(4)]
        return float(min(sides))

    def flat(self, x, y, h):
        """Where a point h cm above the table is put when it is taken to lie ON the table (no height correction)."""
        C = self.C
        k = C[2] / (C[2] - h)
        return (C[0] + (x - C[0]) * k, C[1] + (y - C[1]) * k)


def four_point(cam_c, h, points):
    """Jay-Sun's four points: how far a tag h cm up is put off with no height correction, for a camera lens at cam_c.
    {P: (in off, cm off)}; always away from the camera."""
    out = {}
    C = np.asarray(cam_c, float)
    for p in JAY_IN:
        if p not in points:
            continue                     # (e14) its own spots (no stickers): Jay-Sun's four points are not here
        x, y = points[p]
        k = C[2] / (C[2] - h)
        fx, fy = C[0] + (x - C[0]) * k, C[1] + (y - C[1]) * k
        d = math.hypot(fx - x, fy - y)
        out[p] = (d / 2.54, d)
    return out


def sim_yaw_spec(spec):
    """(11-05h) MIRLT_SIM_TAG_YAW: "180" (every tag turned 180 deg on its robot) or "2=180,3=90" (per tag id): the
    tag reads DEG less than the nose (as tag_cam's tag_yaw draws it), so the heading check finds an offset of +DEG."""
    spec = (spec or "").strip()
    if not spec:
        return 0.0, {}
    if "=" not in spec:
        return float(spec), {}
    by = {}
    for part in spec.split(","):
        k, _, v = part.partition("=")
        if k.strip():
            by[int(k)] = float(v)
    return 0.0, by


def arm_safety(rb, safe_edge, log=say):
    """(11-05h, Nick 5 Nov 20:46) The robot-side safety layers (observe.safety_settings: the tilt stop, the edge
    guard's bottom-zone drop, lift) set and read back with `get` (sent again once). (ok, words)."""
    import observe as OB
    want = OB.safety_settings(safe_edge)
    got = {}
    for k, v in want.items():
        ok, g = rb.set_checked(k, v)
        got[k] = g
    bad = OB.check_safety(want, got)
    words = ", ".join("%s %g" % (k, got[k]) for k in want if got.get(k) is not None)
    if bad:
        return False, "; ".join("%s sent %g, read %s" % (k, v, g) for k, v, g in bad)
    log("  %s: safety layers on and read back: %s" % (rb.name, words))
    return True, words


def heading_nudge(rb, eye, tag, pose, problems, log=say, stop=None, settle_s=SETTLE_S, measure=None):
    """(11-05h, shared: calibration, the missions' wiggle-first, experiments) The roof tag's heading against the
    robot's nose, observed: a NUDGE_CM straight drive, only when clear both ways (observe.nudge_clear: a wrong guess
    cannot take it over an edge; a robot facing an edge turns a quarter first), and the direction the camera saw it
    move. eye.yaw_off[tag] is set (every later camera heading of that tag corrected). pose: (x, y, tag heading) as the
    camera reads it now; problems(a, b) -> reasons. Returns (offset deg, words) or raises RunEnd with plain words."""
    import observe as OB
    measure = measure or (lambda: eye.measure(tag))
    ok, base = OB.nudge_clear(pose, problems, OB.NUDGE_CM)
    if not ok:
        raise _stage(RunEnd("the heading check needs %.0f cm clear both ways from where %s stands (ahead and behind, "
                            "or to both sides): move it nearer the middle of the table and start again" % (
                                OB.NUDGE_CM, rb.name)), "clear")
    if base:
        r = rb.leg("turn", base, on_tick=stop)
        time.sleep(settle_s)
        m = measure()
        if m is None:
            raise _stage(RunEnd("the camera lost %s's tag during the heading check's quarter turn" % rb.name), "turn")
        pose = m[:3]
    r = rb.leg("go", OB.NUDGE_CM, on_tick=stop)
    time.sleep(settle_s)
    c1 = measure()
    if c1 is None:
        raise _stage(RunEnd("the camera lost %s's tag during the heading check: nothing else moved" % rb.name), "go",
                     pose)
    got = OB.heading_offset(pose, c1)
    if got is None:
        raise _stage(RunEnd("the heading check moved %s's tag only %.1f cm of %.0f (%s %s): stopped" % (
            rb.name, math.dist(c1[:2], pose[:2]), OB.NUDGE_CM, r.get("result"), r.get("info"))), "short", c1[:3])
    use, raw, mv = got
    eye.yaw_off = dict(getattr(eye, "yaw_off", {}) or {})
    eye.yaw_off[tag] = wrap180(eye.yaw_off.get(tag, 0.0) + use)
    words = "HEADING CHECK %s: moved %.1f cm at %.0f deg from the tag's heading: tag heading offset %+.0f deg%s" % (
        rb.name, mv, raw, use, "" if abs(use) < 1 else "  <- the tag is turned on the robot; every camera heading of "
        "tag %d corrected (observed, not typed)" % tag)
    log(words)
    return use, raw, mv, c1


def _stage(e, stage, pose=None):
    """(e15) Which step of the heading check ended it ("clear" not clear, "turn" lost in its quarter turn, "go" lost
    after the nudge, "short" it hardly moved) and the last pose the camera had (the reference for a retry)."""
    e.stage, e.pose = stage, pose
    return e


HC_RETRIES = 3            # (e15) a heading check whose tag was lost is tried again this many times (each standing and
                          # reading twice as long as the one before) before the remembered offset or motion is used.
                          # CHOSEN. DOE note: on the school table in dim light, retries 1 / 3 / 5 against checks
                          # finished and seconds spent (the evidence: 10 Nov 18:54, the tag lost for > 3 s at (86, 90))
HC_READS_WANTED = 6       # (e15) a retry reads the tag over a window that holds this many of its own measured fixes:
                          # twice the 3 reads Eye.measure needs. Derived from the tag's measured fix interval (dim light,
                          # school 17:46: 2.3 fixes a second -> 2.6 s; lit: about 0.8 s, so MEASURE_S 1.2 s rules)
HC_FULL_SEARCHES = 2      # (e15) a retry stands long enough for the tag bridge to search the whole picture for a lost
                          # tag this many times (tagcore.FULL_EVERY pictures each, at the tag's measured fix interval)


def fix_interval(eye, tag, span_s=10.0, default=0.1):
    """(e15) The tag's measured time between camera fixes over its last span_s of reads (median gap), or default."""
    try:
        with eye.lock:
            rs = [r[0] for r in (eye.hist.get(int(tag)) or [])[-60:]]
    except (AttributeError, TypeError):
        rs = []
    t_now = time.time()
    rs = [t for t in rs if t_now - t <= span_s]
    if len(rs) < 3:
        return default
    gaps = sorted(b - a for a, b in zip(rs, rs[1:]) if b > a)
    return float(gaps[len(gaps) // 2]) if gaps else default


def heading_check_patient(rb, eye, tag, pose, problems, log=say, stop=None, measure=None, remembered=None,
                          stroke_heading=None, tries=HC_RETRIES):
    """(e15, school 10 Nov 18:54) The heading check that does not leave the robot on its RAW tag heading:
      1. heading_nudge as before;
      2. the tag lost: tried again where the tag reads: it stands longer (the bridge searches the whole picture for a
         lost tag every FULL_EVERY pictures) and the tag is read over a longer window (HC_READS_WANTED of its own
         measured fixes), each try twice as long; a robot that stopped where its tag is not read backs the nudge out
         to where it read and nudges the other way (backwards);
      3. still not measured: the REMEMBERED offset (tag_heading_store, earlier runs) is used as a starting guess;
      4. nothing remembered: the direction of the robot's own finding stroke (motion_find) against the tag's heading,
         as a starting guess;
      5. else unknown: the run measures it by a watched crawl before anything turns it (run_plan straighten_start).
    eye.yaw_off[tag] holds the result (what is added to the tag's camera heading). Returns {"state": "measured" |
    "remembered" | "motion" | "unknown", "offset": total deg or None, "how": words, "pose": the last camera pose or
    None, "raw": seen deg, "moved": cm}. Never raises for a lost tag: only a stop asked (stop()) ends it."""
    import observe as OB
    import tagcore as TC
    measure0 = measure or (lambda: eye.measure(tag))
    eye.yaw_off = dict(getattr(eye, "yaw_off", {}) or {})

    def done(state, how, p=None, raw=None, mv=None):
        return {"state": state, "offset": eye.yaw_off.get(tag) if state != "unknown" else None, "how": how,
                "pose": p, "raw": raw, "moved": mv, "stage": stage}

    stage, ref = "none", pose
    loss = float(getattr(eye, "sim_hc_loss", 0.0) or 0.0)
    if loss and pose is not None and hasattr(eye, "gaps"):
        # (e15, simulator) the school's 10 Nov 18:54: the camera loses the tag just as the heading check's nudge starts
        eye.gaps.append((time.time() - eye.t_made + 0.3, loss))
        eye.sim_hc_loss = 0.0
        log("  (simulator: the camera loses the tags for %.0f s from the heading check's nudge on, as at school 10 Nov "
            "18:54)" % loss)
    if pose is not None:
        try:
            use, raw, mv, c1 = heading_nudge(rb, eye, tag, pose, problems, log=log, stop=stop, measure=measure0)
            return done("measured", "heading check", c1, raw, mv)
        except RunEnd as e:
            stage, ref = getattr(e, "stage", "go"), getattr(e, "pose", None) or pose
            log("  %s: heading check not done (%s): measured again where its tag reads (never driven on the raw tag "
                "heading)" % (rb.name, e))
    out_cm = OB.NUDGE_CM if stage == "go" else 0.0
    gap = fix_interval(eye, tag)
    for k in range(1, (tries if stage != "clear" else 0) + 1):
        if stop is not None and stop():
            break
        stand = max(SETTLE_S, HC_FULL_SEARCHES * TC.FULL_EVERY * gap) * 2 ** (k - 1)
        win = max(MEASURE_S, HC_READS_WANTED * gap) * 2 ** (k - 1)
        log("  %s: HEADING CHECK AGAIN (%d of %d): it stands %.1f s, then its tag is read over %.1f s (its fixes came "
            "every %.2f s)" % (rb.name, k, tries, stand, win, gap))
        time.sleep(stand)
        m2 = eye.measure(tag, win)
        if m2 is not None and out_cm and ref is not None:
            got = OB.heading_offset(ref, m2[:3])
            if got is not None:
                use, raw, mv = got
                eye.yaw_off[tag] = wrap180(eye.yaw_off.get(tag, 0.0) + use)
                log("HEADING CHECK %s (read again after %.1f s): moved %.1f cm at %.0f deg from the tag's heading: tag "
                    "heading offset %+.0f deg%s" % (rb.name, stand, mv, raw, use, "" if abs(use) < 1 else
                                                    "  <- the tag is turned on the robot (observed, not typed)"))
                return done("measured", "heading check, read again after %.1f s" % stand, m2[:3], raw, mv)
        if m2 is None and out_cm:
            # stopped where its tag is not read: back the nudge out (the way it came was checked clear) to where it
            # read, and measure there
            log("  %s: its tag is not read where the nudge stopped: it backs %.0f cm to where its tag read" % (
                rb.name, out_cm))
            rb.leg("go", -out_cm, on_tick=stop)
            out_cm = 0.0
            time.sleep(stand)
            m2 = eye.measure(tag, win)
        if m2 is None:
            continue
        ref = m2[:3]
        sign = -1.0 if k % 2 == 0 else 1.0        # every second try the other way (the far side may read better)
        try:
            got, c1, st = _nudge_dir(rb, eye, tag, ref, problems, sign, stand, lambda: eye.measure(tag, win), stop)
        except RunEnd as e:
            st, got, c1 = getattr(e, "stage", "go"), None, None
            log("  %s: %s" % (rb.name, e))
        if got is not None:
            use, raw, mv = got
            eye.yaw_off[tag] = wrap180(eye.yaw_off.get(tag, 0.0) + use)
            log("HEADING CHECK %s (try %d, %s): moved %.1f cm at %.0f deg from the tag's heading: tag heading offset "
                "%+.0f deg%s" % (rb.name, k, "backwards" if sign < 0 else "forwards", mv, raw, use,
                                 "" if abs(use) < 1 else "  <- the tag is turned on the robot (observed, not typed)"))
            return done("measured", "heading check, try %d" % k, c1, raw, mv)
        out_cm = sign * OB.NUDGE_CM if st == "go" else 0.0
        if st == "clear":
            break
    if out_cm:
        rb.leg("go", -out_cm, on_tick=stop)          # back where its tag read
    if remembered is not None:
        eye.yaw_off[tag] = wrap180(float(remembered["offset_deg"]))
        log("  %s: heading check not measured: tag %s's REMEMBERED turn %+.0f deg is used as the starting guess (%d "
            "measurement%s, last %s); the run checks it again on its first moves" % (
                rb.name, tag, eye.yaw_off[tag], int(remembered.get("n") or 0),
                "" if int(remembered.get("n") or 0) == 1 else "s", remembered.get("last_seen")))
        return done("remembered", "remembered (%s, %s)" % (remembered.get("last_seen"), remembered.get("how")))
    if stroke_heading is not None:
        m3 = eye.measure(tag, max(MEASURE_S, HC_READS_WANTED * gap))
        if m3 is not None:
            raw_tag = wrap180(m3[2] - eye.yaw_off.get(tag, 0.0))
            eye.yaw_off[tag] = OB.snap(wrap180(float(stroke_heading) - raw_tag))
            log("  %s: heading check not measured, nothing remembered: its finding stroke moved at %.0f deg, its tag "
                "reads %.0f deg: tag turned %+.0f deg taken as the starting guess (motion); checked again on its "
                "first moves" % (rb.name, stroke_heading, raw_tag, eye.yaw_off[tag]))
            return done("motion", "the finding stroke's direction", m3[:3])
    eye.yaw_off.pop(tag, None)
    log("  %s: HEADING NOT KNOWN YET: its tag's turn on the robot was not measured and nothing is remembered: the run "
        "measures it by a short watched crawl before anything turns it" % rb.name)
    return done("unknown", "not measured")


def _nudge_dir(rb, eye, tag, pose, problems, sign, settle_s, measure, stop=None):
    """(e15) One nudge forwards (sign +1) or backwards (-1), clear both ways first (a quarter turn when facing an edge).
    Returns (heading_offset result or None, the camera pose after, stage)."""
    import observe as OB
    ok, base = OB.nudge_clear(pose, problems, OB.NUDGE_CM)
    if not ok:
        return None, None, "clear"
    if base:
        rb.leg("turn", base, on_tick=stop)
        time.sleep(settle_s)
        m = measure()
        if m is None:
            return None, None, "turn"
        pose = m[:3]
    rb.leg("go", sign * OB.NUDGE_CM, on_tick=stop)
    time.sleep(settle_s)
    c1 = measure()
    if c1 is None:
        return None, None, "go"
    got = OB.heading_offset(pose, c1, sign=sign)
    return got, c1[:3], ("short" if got is None else "ok")


class SimEye(Eye):
    """The simulator's camera: fleet_sim's true poses (its fixes, no noise) -> what the school's one camera would report
    for the robot's roof tag: only where its tag is big enough in the picture (READ_PX), position noise growing with
    range, and with (bridge as built) or without the tag-height correction. The simulator's robot -> tag id mapping is
    the simulator's own truth: the run never reads it (it binds by the wiggle)."""

    def __init__(self, folder, port, cam, sim_tags, tag_h=6.19, tag_cm=8.0, correct=True, seed=3, decoys=(), gaps=()):
        self.cam, self.sim_tags, self.h, self.s, self.correct = cam, dict(sim_tags), tag_h, tag_cm, correct
        self.t_made = time.time()
        self.gaps = [tuple(float(v) for v in g.split(":")) for g in gaps]   # "T:DUR": the tag unread (a hand over it)
        self.rng = random.Random(seed)
        self.unread = 0
        self.sim_hc_loss = float(os.environ.get("MIRLT_SIM_HC_LOSS", "0") or 0)   # (e15) the heading check's tag lost
        self.sim_yaw, self.sim_yaw_by = sim_yaw_spec(os.environ.get("MIRLT_SIM_TAG_YAW", ""))   # (11-05f/h) tags
        #                                                    turned on the robot: "180" every tag, or "2=180,3=90"
        super().__init__(folder, port)
        for d in decoys:    # tests: a tag lying still on the table (a spare), id, x, y, th
            threading.Thread(target=self._decoy, args=(d,), daemon=True).start()

    def _decoy(self, d):
        tid, x, y, th = d
        while not self.stop_.is_set():
            with self.lock:
                self.latest[tid] = (time.time(), x + self.rng.gauss(0, 0.2), y + self.rng.gauss(0, 0.2), th)
                self.hist.setdefault(tid, []).append(self.latest[tid])
            time.sleep(0.1)

    def transform(self, msg):
        """(e14) The simulator's true pose of every robot is kept (truth_now: the simulated cameras' pictures for robot
        finding by motion are drawn from it), and which robots' tags were NOT read just now (unread_now: drawn with a
        plain roof)."""
        name = msg.get("robot")
        try:
            self.__dict__.setdefault("truth", {})[name] = (float(msg["x"]), float(msg["y"]), float(msg["th"]))
        except (KeyError, TypeError, ValueError):
            pass
        out = self._transform(msg)
        un = self.__dict__.setdefault("unread_set", set())
        if out is None:
            un.add(name)
        else:
            un.discard(name)
        return out

    def blackout(self, t, dur):
        """(e14) The picture black from t s after the camera starts, for dur s: no tag reads, and the bridge's "dark"
        then (after dark_watch's SETTLE_S) "light_back" messages, as the tag bridge's DarkWatch sends them."""
        self.gaps.append((float(t), float(dur)))

        def run():
            import dark_watch as DW
            while time.time() - self.t_made < t and not self.stop_.is_set():
                time.sleep(0.1)
            say("THE PICTURE WENT DARK on camera overhead (simulated blackout): the camera is adjusting to the light")
            self.camera_state, self.dark_t = ("dark", time.time()), time.time()
            while time.time() - self.t_made < t + dur + DW.SETTLE_S and not self.stop_.is_set():
                time.sleep(0.1)
            say("THE PICTURE IS BACK on camera overhead after %.0f s (simulated)" % dur)
            self.camera_state = ("light_back", time.time())
        threading.Thread(target=run, daemon=True).start()

    def truth_now(self):
        return dict(getattr(self, "truth", {}) or {})

    def unread_now(self):
        return set(getattr(self, "unread_set", set()) or set())

    def _transform(self, msg):
        name = msg.get("robot")
        if name not in self.sim_tags:
            return None
        if self.sim_tags.get(name) is None or int(self.sim_tags[name]) < 0:
            return None                  # (e14) a robot with no roof tag (a plain roof): never read
        for (dx, dy, dr, _f) in getattr(self, "darks", []) or []:
            if math.hypot(float(msg["x"]) - dx, float(msg["y"]) - dy) <= dr:
                return None              # (e14) a dark patch: the tag does not read there (the motion may still see it)
        el = time.time() - self.t_made
        if any(t <= el < t + d for t, d in self.gaps):
            return None
        # (11-05c) "X,Y,R:DUR": a spot the camera cannot read (glare, the far end of its view): the tag is unread while
        # the robot is within R cm of (X, Y), for DUR s from the first time it comes there (then it reads again)
        for g in getattr(self, "gaps_at", []):
            inside = math.hypot(float(msg["x"]) - g["x"], float(msg["y"]) - g["y"]) <= g["r"]
            if g.get("t") is None and inside:
                g["t"] = el
            if inside and g.get("t") is not None and g["t"] <= el < g["t"] + g["dur"]:
                return None
        x, y, th = float(msg["x"]), float(msg["y"]), float(msg["th"])
        px = self.cam.tag_px(x, y, self.h, self.s, th)
        if px is None or px < READ_PX:
            self.unread += 1
            return None
        rng_cm = float(np.linalg.norm(np.array([x, y, self.h]) - self.cam.C))
        sd = 0.08 + 0.0012 * rng_cm          # chosen: ~0.4 cm at 3 m (sub-pixel corners, one camera). Not measured
        if not self.correct:
            x, y = self.cam.flat(x, y, self.h)
        return {"type": "fix", "robot": "tag%d" % self.sim_tags[name], "x": x + self.rng.gauss(0, sd),
                "y": y + self.rng.gauss(0, sd), "th": th + self.rng.gauss(0, 0.8 + 0.002 * rng_cm), "sigma": sd,
                "source": "sim one camera", "t": msg.get("t")}


# =============================================================================================== binding
def feed_refusal(eye, wait_s=None):
    """(e13, the 10 Nov 11:48 school calibration: 16 s of no pictures was reported as "no roof tag is seen ... check
    the robot") Before blaming a robot or a tag: does every camera send new pictures (camera_feed.check on the
    bridge's frames.json)? If not: BindError with the camera's plain words. A simulated eye (no bridge) is not
    checked."""
    import camera_feed as CF
    folder = getattr(eye, "folder", None)
    bd = os.path.join(folder, "bridge") if folder else None
    if isinstance(eye, SimEye) or not bd or not os.path.isdir(bd):
        return
    ok, words, _d = CF.check(bd, wait_s=CF.WAIT_S if wait_s is None else wait_s)
    if not ok:
        raise BindError(words)
    say(words)


def bind_by_wiggle(robots, eye, wiggle_deg=90.0, pause_s=1.0, log=say, moved_deg=None, expect=None, check_deg=None,
                   notes=None):
    """Each robot (in turn) does the sync wiggle; the tag whose heading follows it is bound. {name: tag id} or raises
    BindError with plain words. robots: [Robot]. moved_deg: a tag 'moves' when its heading changes more than this
    (default half the wiggle); a robot's tag must turn within 40 % of the wiggle and come back within 25 deg.
    (5 Nov 11:00, Nick: "pin it because the tags won't change") expect: {robot name: its pinned tag}: a pinned robot
    does the check wiggle (check_deg) and its pinned tag must be the one that turns; when another tag turns instead it
    is re-bound to that one, said loudly. notes: {name: what happened} filled in."""
    md0 = moved_deg
    out = {}
    for rb in robots:
        pin = (expect or {}).get(rb.name)
        wiggle_deg_ = float(check_deg) if (pin is not None and check_deg) else wiggle_deg
        moved_deg = md0 if md0 is not None else wiggle_deg_ / 2.0
        before = {}
        t0 = time.time()
        time.sleep(1.0)
        # (11-10i, school 15:00: AL-EB54 at P15, where tag 4 reads only now and then, was refused after 2 s) a standing
        # robot cannot drift: look over longer windows (2, 4, 8, 15 s) before calling the table empty
        for w in (2.0, 4.0, 8.0, STILL_MEASURE_MAX_S):
            if w > 2.0:
                log("  no roof tag read yet: looking over %.0f s (a tag far from the camera reads only now and then)"
                    % w)
                time.sleep(w - 2.0)
            for i in eye.seen_ids(w):
                rs = eye.window(i, time.time() - w, time.time())
                if rs:
                    before[i] = rs[-1][3]
            if before:
                break
        if not before:
            feed_refusal(eye)                  # (e13) a camera sending no pictures is said as that, never the robot's
            raise BindError("no roof tag is seen by the camera (nothing to bind %s to). Check that the robot is in "
                            "zones 1-2 and its tag faces up, and that the camera's picture is sharp." % rb.name)
        def fresh(since, wait):
            # (11-10i) each tag's newest heading read after `since`, waiting (standing still) up to `wait` s for one
            t_end = time.time() + wait
            while time.time() < t_end and any((eye.last(i) or (0,))[0] < since for i in before):
                time.sleep(0.1)
            return {i: (eye.last(i) or (0, 0, 0, before[i]))[3] for i in before}
        r1 = rb.leg("turn", wiggle_deg_)
        t_r1 = time.time()
        mid = fresh(t_r1, pause_s)
        time.sleep(max(0.0, pause_s - (time.time() - t_r1)))
        mid2 = fresh(t_r1, STILL_MEASURE_MAX_S)
        r2 = rb.leg("turn", -wiggle_deg_)
        t_r2 = time.time()
        time.sleep(0.6)
        after = fresh(t_r2, STILL_MEASURE_MAX_S)
        if r1["result"] != "done" or r2["result"] != "done":
            raise BindError("%s did not do its wiggle (%s %s / %s %s)" % (rb.name, r1["result"], r1["info"],
                                                                         r2["result"], r2["info"]))
        moved = []
        for i in before:
            # (11-05h) the mean of the two mid readings taken round the circle: a plain mean of +179 and -178 is 0
            # (the full fleet suite: a tag heading 92 -> 180 read as -91 for +90, not bound)
            d1 = wrap180(mid[i] + 0.5 * wrap180(mid2[i] - mid[i]) - before[i]) if abs(wrap180(mid[i] - mid2[i])) < 20 else \
                wrap180(mid2[i] - before[i])
            back = wrap180(after[i] - before[i])
            log("  tag %d: turned %+.0f deg with %s's wiggle, %+.0f deg from where it started after it" % (
                i, d1, rb.name, back))
            if abs(d1) > moved_deg:
                ok = abs(d1 - wiggle_deg_) <= 0.4 * abs(wiggle_deg_) and abs(back) <= 25
                moved.append((i, ok, d1))
        good = [i for i, ok, _ in moved if ok]
        if not moved:
            raise BindError("no roof tag moved when %s did its wiggle (tags seen: %s)%s. Is that robot in the camera's "
                            "view, with its tag on its roof?" % (rb.name, ", ".join(map(str, before)),
                                                                 "; it is pinned to tag %d: PIN CHECK FAILED" % int(pin)
                                                                 if pin is not None else ""))
        if len(moved) > 1:
            raise BindError("%d tags moved when %s did its wiggle (%s): cannot tell which one is on it. Only one robot "
                            "may move at a time; check nobody moved a tag." % (len(moved), rb.name,
                                                                                 ", ".join(str(m[0]) for m in moved)))
        if not good:
            raise BindError("tag %d moved with %s's wiggle but not as commanded (%+.0f deg for %+.0f): not bound"
                            % (moved[0][0], rb.name, moved[0][2], wiggle_deg_))
        if good[0] in out.values():
            raise BindError("tag %d moved with two robots' wiggles" % good[0])
        out[rb.name] = good[0]
        if pin is None:
            note = "bound by the wiggle (no pin yet)"
        elif int(pin) == good[0]:
            note = "pin checked: its pinned tag %d turned with it" % good[0]
        else:
            note = "PIN CHECK FAILED: pinned to tag %d, but tag %d turned with it: re-bound to tag %d" % (
                int(pin), good[0], good[0])
            log("!!! %s: %s. Was a roof tag moved to another robot? The pin is changed to tag %d." % (
                rb.name, note, good[0]))
        log("  %s: %s" % (rb.name, note))
        if notes is not None:
            notes[rb.name] = note
    return out


class BindError(Exception):
    pass


def compass_check(link, rb, log=say):
    """(5 Nov) A robot woken from `sleep` has restarted: it refuses `turn` / `go` legs until its compass check has run
    (firmware 2026-10-25c), so a wiggle would be refused. A `turn 0` leg asks; if refused for that reason, the
    calibration run with no spin and no roll (the compass check's few-degree wiggle alone, run_plan's compass_only) is
    run, then it is asked again. (ok, words)."""
    r = rb.leg("turn", 0.0, timeout=10.0)
    if r["result"] == "done":
        return True, "already checked"
    info = r.get("info") or ""
    if "compass" not in info and "calibrate" not in info:
        log("  %s: the check leg answered %s %s (carrying on)" % (rb.name, r["result"], info))
        return False, "%s %s" % (r["result"], info)
    import calib_room as CRM
    log("  %s restarted since its last compass check (woken from sleep): the compass check now (a few degrees of "
        "wiggle, no spin, no roll; about 10 s)" % rb.name)
    link.send(rb.ip, "set calib_spin_deg 0")
    link.send(rb.ip, CRM.caps_cmd([(d, 0.0) for d in CRM.CAP_DEGS]))
    time.sleep(0.3)
    k = link.mark()
    link.send(rb.ip, "calibrate")
    link.wait_for(lambda n, l: l if n == rb.name and l.startswith("D,RUNEND,") else None, k, 60.0)
    link.send(rb.ip, "set calib_spin_deg 720")
    r = rb.leg("turn", 0.0, timeout=10.0)
    ok = r["result"] == "done"
    log("  %s: compass check %s" % (rb.name, "done" if ok else "did NOT take (%s %s)" % (r["result"], r["info"])))
    return ok, "compass check run" if ok else "compass check failed"


def site_name_for(site, wifi_name, ip):
    """The site's name for a robot found on the WiFi (a site robot listing its ip or wifi name), else the WiFi name."""
    for r in (site or {}).get("robots") or []:
        if (r.get("ip") and r.get("ip") == ip) or (r.get("wifi") and r.get("wifi") == wifi_name):
            return r["name"]
    return wifi_name


def fixed_binding(site, names):
    """The old way (switch --binding fixed): the site's tags.ids by robot name."""
    ids = ((site or {}).get("tags") or {}).get("ids") or {}
    out = {n: int(ids[n]) for n in names if n in ids}
    missing = [n for n in names if n not in ids]
    if missing:
        raise BindError("--binding fixed: the site's tags.ids does not list %s (it lists %s)" % (
            ", ".join(missing), ", ".join("%s=%s" % kv for kv in ids.items())))
    return out


# ======================================================== firmware, WiFi names, pins, calibrated robots (5 Nov 11:00)
FW_RX = re.compile(r"\(firmware ([0-9]{4}-[0-9]{2}-[0-9]{2}[a-z]?)")


def firmware_of(rb, timeout=2.5):
    """The firmware version in the robot's `status` line ("" when it does not say: firmware before 2026-10-28)."""
    r = rb.ask("status", lambda l: FW_RX.search(l) if l.startswith("M,status ") else None, timeout)
    return r.group(1) if r else ""


def firmware_ok(ver, minv):
    """Version strings are dates with a letter (2026-11-05a): they sort as text. minv empty = no check."""
    return (not minv) or (bool(ver) and ver >= minv)


def firmware_refusal(name, ver, minv):
    return ("%s is on firmware %s; this run needs %s or newer. Reflash it with the repo's firmware/home_lab (runbook: "
            "Reflash a robot), then start again" % (name, ver or "older than 2026-10-28 (its status does not say)", minv))


def wifi_name_of(link, rb):
    """The robot's WiFi name (AL-xxxx) from `id` (firmware 2026-11-05a, answers it even after `name`); older: the
    name it answers with when that is a WiFi name, else None."""
    r = rb.ask("id", lambda l: l if l.startswith("M,id ") else None, 1.5, tries=2)
    m = re.search(r" wifi (\S+)", r or "")
    return m.group(1) if m else (rb.name if rb.name.startswith("AL-") else None)


def state_path(a_or_dir, name):
    """Where this site's records live: the Pi's site folder (conf_dir), a dry run's own sim folder (never the Pi's)."""
    d = a_or_dir if isinstance(a_or_dir, str) else getattr(a_or_dir, "state_dir", "")
    return os.path.join(d, name) if d else ""


def load_json(p, default):
    try:
        with open(p) as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


def save_json(p, obj):
    if not p:
        return
    os.makedirs(os.path.dirname(p), exist_ok=True)
    with open(p + ".tmp", "w") as f:
        json.dump(obj, f, indent=1, sort_keys=True)
    os.replace(p + ".tmp", p)


PINS_FILE = "robot_pins.json"          # {WiFi name: {"tag": id, "slot": "Alpha", "since": ..., "checked": ...}}
CALIBRATED_FILE = "calibrated_robots.json"   # {WiFi name: {"first": ..., "last": ..., "values": {...}, "how": ...}}


def load_pins(site, path):
    """The pins: the site's tags.pins (typed, may be empty) under the Pi's robot_pins.json (what the wiggles found)."""
    out = {}
    for w, v in (((site or {}).get("tags") or {}).get("pins") or {}).items():
        out[w] = dict(v, source="site")
    for w, v in (load_json(path, {}) if path else {}).items():
        out[w] = dict(out.get(w, {}), **v)
    return out


def update_pins(path, pins, wifi_tag, wifi_slot=None):
    """Record what a wiggle found: {wifi: tag} (and {wifi: slot}); returns the pins as written."""
    now = dt.datetime.now().isoformat(timespec="seconds")
    cur = load_json(path, {}) if path else {}
    for w, t in wifi_tag.items():
        if not w:
            continue
        e = dict(cur.get(w) or {})
        if e.get("tag") != int(t):
            e["since"] = now
        e.update(tag=int(t), checked=now)
        if not e.get("slot"):         # a slot, once given, stays (the pin); robot_pins.json is edited to change it
            e["slot"] = (pins.get(w) or {}).get("slot") or (wifi_slot or {}).get(w) or None
            if not e["slot"]:
                del e["slot"]
        cur[w] = e
    save_json(path, cur)
    return cur


# =============================================================================================== the fit
def stop_overrun(rows, w, ws0):
    """(11-05h, school 5 Nov 19:34) How far past its commanded length a leg really ends, per drive speed, once the new
    wheel_scale is in: rows [(belief cm, camera cm, speed cm/s, commanded cm)] from phase A (every finished go leg of
    4 cm or more, the 5 cm finals included). With wheel_scale = w the robot stops when its belief reaches the
    commanded length less its usual shortfall s (the stop tolerance, in belief cm), so it really covers
    (commanded - s) and then slides on: coast(v) = camera - w x counted, measured per speed. Overrun(v) = coast(v) - s.
    The real run: 0.32 cm at 5 cm/s and 0.49 cm at 10 cm/s predicted from phase A; phase C measured 0.31 and 0.52.
    A fixed few mm per leg is not a scale error: divided by the leg's length it looked like "+1 cm/m after the fit"."""
    by = {}
    short = []
    for bel, cam, v, cmd in rows:
        counted = bel / ws0
        by.setdefault(round(float(v), 1), []).append(cam - w * counted)
        short.append(cmd - bel)
    if not short:
        return {}
    s = float(np.median(short))
    over = {("%g" % v): round(float(np.median(c)) - s, 3) for v, c in by.items() if len(c) >= OVERRUN_MIN_N}
    return {"stop_shortfall_cm": round(s, 3), "overrun_cm": over,
            "overrun_n": {("%g" % v): len(c) for v, c in by.items()}}


ROLL_OWN_MIN = 0.5       # (e16) a drive whose own count is under this share of the camera's length is not a measurement
                         # of the wheels. Not a tuning: real legs sit at 0.97-1.03 of the camera's; a lost report gives 0


def fit_values(turn_rows, still_rows, roll_rows, old, overrun_rows=None):
    """From phase A's measured legs: wheel_scale, gyro_scale, gyro_drift_dps with 1-sigma SEs.
    turn_rows: [(belief dtheta deg, dt s, camera dtheta deg)]; still_rows: [(belief dtheta, dt)];
    roll_rows: [(belief length cm, camera length cm)]; old: the values the robot used (to undo them).
    Firmware model (motion.h, corr.h): belief heading = gs x raw - gd x t; real = g x raw - d x t (what we want);
    belief length = ws x counted; real = w x counted."""
    gs0, gd0, ws0 = old["gyro_scale"], old["gyro_drift_dps"], old["wheel_scale"]
    rows = []
    for db, dt_, dc in turn_rows:
        raw = (db + gd0 * dt_) / gs0
        rows.append((raw, -dt_, dc, 1.0))
    for db, dt_ in still_rows:
        raw = (db + gd0 * dt_) / gs0
        rows.append((raw, -dt_, 0.0, 1.0))
    out = {"n_turns": len(turn_rows), "n_still": len(still_rows), "n_rolls": len(roll_rows)}
    if len(rows) >= 3 and still_rows:
        X = np.array([[r[0], r[1]] for r in rows])
        y = np.array([r[2] for r in rows])
        beta, *_ = np.linalg.lstsq(X, y, rcond=None)
        res = y - X @ beta
        dof = max(1, len(rows) - 2)
        s2 = max(float(res @ res) / dof, 0.05 ** 2)
        cov = np.linalg.inv(X.T @ X) * s2
        out.update(gyro_scale=float(beta[0]), gyro_drift_dps=float(beta[1]), se_gyro_scale=float(math.sqrt(cov[0, 0])),
                   se_gyro_drift_dps=float(math.sqrt(cov[1, 1])), rms_turn_deg=float(math.sqrt(np.mean(res ** 2))))
    # (e16, school 17:02) a drive whose own count is under half what the camera saw was not measured by the robot (a
    # lost "started" report gives 0 cm): one such leg took the fit to 0.73 +- 0.20. Left out, and counted
    if roll_rows:
        ok_ = [r for r in roll_rows if r[0] / ws0 >= ROLL_OWN_MIN * r[1]]
        out["rolls_unmeasured"] = len(roll_rows) - len(ok_)
        roll_rows = ok_
        out["n_rolls"] = len(roll_rows)
    if roll_rows:
        # camera length = w x counted + coast: the wheels still turn a little after the leg's `done` record (the robot
        # brakes after it), the same few mm on every leg; with legs of several lengths both are found (Claude's model)
        b = np.array([r[0] / ws0 for r in roll_rows])     # counted cm
        c = np.array([r[1] for r in roll_rows])
        if len(b) >= 4 and np.ptp(b) > 10:
            X = np.stack([b, np.ones_like(b)], 1)
            beta, *_ = np.linalg.lstsq(X, c, rcond=None)
            res = c - X @ beta
            s2 = max(float(res @ res) / max(1, len(b) - 2), 0.1 ** 2)
            cov = np.linalg.inv(X.T @ X) * s2
            w, se, coast = float(beta[0]), float(math.sqrt(cov[0, 0])), float(beta[1])
        else:
            w = float(b @ c / (b @ b))
            res = c - w * b
            s2 = max(float(res @ res) / max(1, len(b) - 1), 0.1 ** 2)
            se, coast = float(math.sqrt(s2 / (b @ b))), 0.0
        out.update(wheel_scale=w, se_wheel_scale=se, coast_cm=coast,
                   rms_roll_cm=float(math.sqrt(np.mean(res ** 2))), counted_cm=float(b.sum()))
    if overrun_rows and "wheel_scale" in out:
        out.update(stop_overrun(overrun_rows, out["wheel_scale"], ws0))
    verdict = {}
    for k in PARAMS:
        if k not in out:
            verdict[k] = "not enough data"
            continue
        lo, hi = BOUNDS[k]
        pm = 1.96 * out["se_" + k]
        if not lo <= out[k] <= hi:
            verdict[k] = "outside %.2f-%.2f: refused" % (lo, hi)
        elif pm > ENOUGH[k]:
            verdict[k] = "not pinned down: +-%.4f (need +-%.4f)" % (pm, ENOUGH[k])
        else:
            verdict[k] = "ok"
    out["verdict"] = verdict
    return out


def tag_offset(samples):
    """The tag's offset from the turning point from in-place turns: [(x, y, th deg)] at rest between turns. Solves
    tag = c + R(th) o for c (turning point) and o (offset in the robot frame: ahead, left). (c, o, rms cm)."""
    if len(samples) < 3:
        return None
    A, b = [], []
    for x, y, th in samples:
        c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
        A.append([1, 0, c, -s])
        b.append(x)
        A.append([0, 1, s, c])
        b.append(y)
    A, b = np.array(A), np.array(b)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    rms = float(math.sqrt(np.mean((A @ sol - b) ** 2)))
    return (float(sol[0]), float(sol[1])), (float(sol[2]), float(sol[3])), rms


# =============================================================================================== the run
class CalibRun:
    def __init__(self, a, site, folder, park_label=None, parent=None):
        os.makedirs(folder, exist_ok=True)
        self.a, self.folder, self.parent = a, folder, parent
        self.S = Site(site, park_label or PARK_SPOTS[0], spots=getattr(a, "spots", None))
        if parent is not None and parent.S.spots_mode == "own":
            # (e14) one robot of several: the run's own spots, made once
            self.S.points, self.S.zones = dict(parent.S.points), dict(parent.S.zones)
            self.S.own_centre = getattr(parent.S, "own_centre", None)
        if getattr(a, "view_x_max", None):
            self.S.view_x = float(a.view_x_max)   # (11-05g) the tag reads only this far up the table in this light
        self.site = site
        if WORKSPACE_ON:
            try:
                w_ = self.S.use_workspace(site, os.path.join(a.state_dir, "..") if a.dry and getattr(a, "state_dir", "")
                                          else None)
                if parent is None:
                    say("PROVEN WORKSPACE: %s" % w_)
            except Exception as e:  # noqa: BLE001  (the typed table as before)
                say("(proven workspace not used: %s)" % e)
        self.events = open(os.path.join(folder, "events.jsonl"), "a")
        self.status = {"version": VERSION, "phase": "starting", "robot": None, "tag": None, "words": "Starting",
                       "started": time.time(), "dry": bool(a.dry), "result": None, "folder": folder}
        self.legs_f = open(os.path.join(folder, "legs.csv"), "w", newline="")
        self.legs = csv.writer(self.legs_f)
        self.legs.writerow(["phase", "tour", "n", "kind", "target", "commanded", "result", "info", "t0", "t1",
                            "cam0_x", "cam0_y", "cam0_th", "cam1_x", "cam1_y", "cam1_th", "bel0_x", "bel0_y", "bel0_th",
                            "bel1_x", "bel1_y", "bel1_th", "dt_s", "cam_len_cm", "bel_len_cm", "cam_turn_deg",
                            "bel_turn_deg", "along_err_cm", "cross_err_cm", "turn_err_deg", "end_miss_cm",
                            "corrected_first", "held_s", "edge_cm_min", "keepout_cm_min", "interrupted",
                            "sent_cm", "speed_cms"])
        self.raw_f = open(os.path.join(folder, "odometry_raw.csv"), "w", newline="")
        self.raw = csv.writer(self.raw_f)
        self.raw.writerow(["t", "x_cm", "y_cm", "th_deg", "robot_ms", "anchor"])
        self.cor_f = open(os.path.join(folder, "odometry_corrected.csv"), "w", newline="")
        self.cor = csv.writer(self.cor_f)
        self.cor.writerow(["t", "x_cm", "y_cm", "th_deg", "robot_ms", "anchor"])
        self.Draw = None             # the robot's own pose in the table frame, anchored ONCE (never camera-corrected)
        self.D = None                # the same, moved to the camera's pose by each safety correction / tour start
        self.n_anchor = 0
        self.offset = (0.0, 0.0)     # the tag's offset from the turning point (ahead, left), cm
        self.closest = {"edge_cm": 1e9, "keepout_cm": 1e9, "view_x_max": -1e9}
        self.stop_flag = threading.Event()
        self.holds = []
        self.battery = []
        self.leg_n = 0
        self._leg_moving = False
        self.reliable = []           # (11-05c) camera poses where the tag read reliably (averaged at stops): (x, y)
        self.recoveries = []
        self._recovered = False

    # ------------------------------------------------------------------------------------------------- bookkeeping
    def ev(self, kind, **kw):
        kw.update(kind=kind, t=round(time.time(), 3))
        self.events.write(json.dumps(kw, default=str) + "\n")
        self.events.flush()

    def set_status(self, **kw):
        self.status.update(kw)
        self.status["updated"] = time.time()
        p = os.path.join(self.folder, "status.json")
        with open(p + ".tmp", "w") as f:
            json.dump(self.status, f, indent=1, default=str)
        os.replace(p + ".tmp", p)
        if self.parent is not None:
            # (5 Nov) one robot of several: Mission Control's card reads the top folder's status
            up = {k: v for k, v in kw.items() if k in ("phase", "robot", "tag", "point", "battery", "fit", "wifi_name",
                                                        "saved", "baseline", "baseline_words")}
            if "words" in kw:
                up["words"] = "%s (robot %d of %d): %s" % (self.name if hasattr(self, "name") else "?",
                                                           self.k + 1, self.n, kw["words"])
            if up:
                self.parent.set_status(**up)

    # ------------------------------------------------------------------------------------------------- poses
    def cam_pose(self, dur=MEASURE_S):
        """The turning point's pose from the camera (tag offset taken off), averaged; None if the tag is not seen."""
        m = self.eye.measure(self.tag, dur)
        if m is None:
            return None
        x, y, th, n = m
        c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
        ox, oy = self.offset
        p = (x - (c * ox - s * oy), y - (s * ox + c * oy), th)
        self.closest["edge_cm"] = min(self.closest["edge_cm"], self.S.edge_dist(p[:2]))
        self.closest["keepout_cm"] = min(self.closest["keepout_cm"], self.S.keepout_dist(p[:2])[0])
        self.closest["view_x_max"] = max(self.closest["view_x_max"], p[0])
        if n >= 8 and (not self.reliable or math.dist(self.reliable[-1], p[:2]) > 1.0):
            self.reliable.append((p[0], p[1]))       # (11-05c) a spot where the tag read well (8+ reads)
            del self.reliable[:-200]
        return p

    def cam_pose_sure(self):
        """(11-10c, school 12:35: Phase A's roll setup got None from cam_pose after the still period and crashed) the
        camera pose, waiting for the tag when it is momentarily not seen (the robot is standing still)."""
        k = 0
        while True:
            # (11-10g) standing still it cannot drift: a longer window each time, never giving up (Ctrl+C / Stop ends)
            if getattr(self, "_ending", False) and k >= 3 and PARK_GIVEUP:
                # (e13) parking after the run's end: the end is not held forever
                raise RunEnd("no camera fix of the roof tag while parking: the robot stays where it is")
            self.wait_light("waiting for its camera fix")       # (e14)
            cam = self.cam_pose(min(STILL_MEASURE_MAX_S, MEASURE_S * (2 ** k)))
            if cam is not None:
                return cam
            if k in (0, 2) or k % 5 == 0:
                say("  the camera has no steady fix of the roof tag right now (the robot stands still, so it cannot "
                    "drift): measuring over a longer window")
            if self.stop_flag.is_set():
                raise RunEnd("stopped (Ctrl+C or Mission Control's Stop)")
            if self.eye.age(self.tag) > 5.0:
                self.hold_until_seen()     # (e13) the never-give-up hold: NEEDS A HAND, motion sightings re-tried
            k += 1

    def odo_step(self, r):
        """After a leg: the robot's own motion added to both odometry tracks (raw and corrected)."""
        if r.get("start") is None or r.get("end") is None or self.D is None:
            return
        rel = relative(r["start"], r["end"])
        self.Draw = compose(self.Draw, rel)
        self.D = compose(self.D, rel)
        t = time.time()
        self.raw.writerow(["%.3f" % t, "%.2f" % self.Draw[0], "%.2f" % self.Draw[1], "%.2f" % self.Draw[2],
                           r["end"][3], 0])
        self.cor.writerow(["%.3f" % t, "%.2f" % self.D[0], "%.2f" % self.D[1], "%.2f" % self.D[2], r["end"][3],
                           self.n_anchor])

    def reanchor(self, cam, why):
        if self.Draw is None:
            self.Draw = tuple(cam)
            self.D = tuple(cam)
            return
        self.n_anchor += 1
        self.ev("pose_corrected", why=why, camera=cam, odometry=self.D)
        self.D = tuple(cam)

    # ------------------------------------------------------------------------------------------------- safety
    def watch(self):
        """During a leg: the cautious policy (tag lost > LOST_HOLD_S -> stop). True = stop sent."""
        if self.a.lost_policy not in ("hold", "recover"):
            return self.stop_flag.is_set()
        if self.stop_flag.is_set():
            return True
        if self.eye.age(self.tag) > LOST_HOLD_S and self._leg_moving:
            self._lost_stop = True
            self._lost_last = self.eye.last(self.tag)       # (11-05c) the last read before it was lost
            return True
        return False

    def dark_recent(self, within_s=120.0):
        e = getattr(self, "eye", None)
        if e is None:
            return False
        return (hasattr(e, "is_dark") and e.is_dark()) or time.time() - float(getattr(e, "dark_t", 0) or 0) < within_s

    def wait_light(self, where=""):
        """(e14, Nick at school 17:2x: the room lights went out, the picture went black, the run ended and the robot
        was put to sleep; minutes later the camera had adjusted) While the bridge says the picture is dark: every robot
        holds still and safe, nothing ends, the run carries on by itself when the picture is back. Returns seconds."""
        e = getattr(self, "eye", None)
        if e is None or not hasattr(e, "is_dark") or not e.is_dark():
            return 0.0
        t0 = time.time()
        w = ("THE PICTURE WENT DARK: the camera is adjusting to the light (room lights off?). %s holds still and safe%s; "
             "it carries on by itself when the picture is back" % (getattr(self, "name", "the robot"),
                                                                    (" (" + where + ")") if where else ""))
        say("!! " + w)
        self.set_status(words=w)
        self.ev("dark_hold", where=where)
        for rb in getattr(self, "robots", None) or ([self.rb] if getattr(self, "rb", None) else []):
            rb.link.send(rb.ip, "stop")
        while e.is_dark():
            if self.stop_flag.is_set():
                raise RunEnd("stopped (Ctrl+C or Mission Control's Stop) while the picture was dark")
            time.sleep(0.2)
        d = time.time() - t0
        say("  the picture is back after %.0f s: carrying on" % d)
        self.ev("dark_back", held_s=round(d, 1))
        host = getattr(self, "parent", None) or self
        host.dark_held = getattr(host, "dark_held", 0.0) + d
        time.sleep(1.0)                 # the tag reads again
        return d

    def hold_until_seen(self):
        giveup = __import__("safety_group").flag("MIRLT_CALIB_GIVEUP") or getattr(self.a, "lost_giveup", False)
        if not giveup:
            return self.hold_never_give_up()     # (e13) the 11-10g wait, plus the motion detector's sightings re-tried
        t0 = time.time()
        self.ev("hold", why="roof tag lost for over %.1f s while driving" % LOST_HOLD_S)
        self.set_status(words="HOLDING: the camera lost the roof tag; waiting for it (up to %.0f s)" % LOST_GIVEUP_S)
        say("HOLD: the roof tag was lost while driving; the robot stopped. Waiting for the camera to see it again ...")
        nxt = LOST_GIVEUP_S
        while True:
            # (11-10g, Nick: never give up) the robot is stopped, so waiting is safe: it waits for the camera (or Stop),
            # saying so every LOST_GIVEUP_S, instead of ending the run (switch: --lost-giveup ends it as before)
            if self.eye.age(self.tag) < 0.3:
                d = time.time() - t0
                self.holds.append(d)
                say("  seen again after %.1f s: carrying on" % d)
                return d
            if self.stop_flag.is_set():
                raise RunEnd("stopped (Ctrl+C or Mission Control's Stop)")
            if time.time() - t0 > nxt:
                if giveup:
                    raise RunEnd("the camera lost the roof tag for over %.0f s: the robot stopped where it is (not "
                                 "parked)" % LOST_GIVEUP_S)
                say("  STILL HOLDING after %.0f s: the camera cannot see %s's roof tag. It is stopped and safe. Move "
                    "it where the camera sees it, or add light; Ctrl+C or Stop ends the run" % (
                        time.time() - t0, getattr(self, "name", "the robot")))
                self.set_status(words="NEEDS A HAND: the camera cannot see %s's roof tag (stopped, safe)" %
                                getattr(self, "name", "the robot"))
                nxt += LOST_GIVEUP_S
            time.sleep(0.1)

    def light_words(self):
        """(e13, school 12:45: the room lights went out) "the room got darker: the camera's brightness dropped from A to
        B (-P %)" from the bridge's brightness.csv (its first rows against its last few), or "" when it did not drop by
        the bridge's own LIGHT_WARN_FRAC."""
        try:
            import tag_bridge as TB
            frac = float(TB.Bridge.LIGHT_WARN_FRAC)
        except Exception:  # noqa: BLE001
            frac = 0.15
        host = getattr(self, "parent", None) or self
        for d_ in (getattr(self, "folder", ""), getattr(host, "folder", "")):
            p_ = os.path.join(d_ or "", "bridge", "brightness.csv")
            if not d_ or not os.path.exists(p_):
                continue
            try:
                with open(p_, newline="") as f:
                    rows = [float(r["mean_grey"]) for r in csv.DictReader(f) if r.get("mean_grey")]
            except (OSError, ValueError, KeyError):
                return ""
            if len(rows) < 20:
                return ""
            b0 = sorted(rows[:30])[len(rows[:30]) // 2]
            b1 = sorted(rows[-10:])[5]
            if b0 > 0 and (b0 - b1) / b0 >= frac:
                return ("the room got darker: the camera's brightness dropped from %.0f to %.0f (-%.0f %%). Turn the "
                        "lights back on" % (b0, b1, 100.0 * (b0 - b1) / b0))
            return ""
        return ""

    def wiggle_seen(self):
        """(e14) The find-me wiggle with every camera watching (motion_find.locate_wiggle on the pictures, as the
        robot finding does): (x, y) where it was seen, True when it wiggled and nobody saw it, None when there are no
        pictures to watch (the plain wiggle then). Switch: MIRLT_CAMERA_DISCOVERY=off."""
        import motion_find as MF
        if not MF.enabled() or getattr(self, "rb", None) is None:
            return None
        if getattr(self, "_mcams", None) is None:
            try:
                self._mcams, _cl = self.motion_cams()
            except Exception:  # noqa: BLE001
                self._mcams = []
        if not self._mcams:
            return None
        import camera_discovery as CD

        def turn_ticks(deg, on_tick, n):
            r = CD.leg_turner({self.rb.name: self.rb}, TURN_DPS)(self.rb.name, deg, on_tick, n)
            if isinstance(r, dict):
                self.odo_step(r)
            return r
        try:
            w = MF.locate_wiggle(self._mcams, turn_ticks)
        except Exception as e:  # noqa: BLE001
            say("  (the wiggle was not watched: %s)" % e)
            return None
        if w is None:
            return True
        self.ev("hold_seen_by_pictures", x=round(w["where"][0], 1), y=round(w["where"][1], 1), sigma=w["sigma"])
        return tuple(w["where"])

    def hold_creep(self, hs):
        """(e18, never still) A creep while held: toward the nearest readable cell the map knows (map_target), a turn
        toward it and 2-4 cm (less with a bigger error), only where its circle plus its error stays clear of every edge,
        outlet and keep-out at the end."""
        try:
            if hs is None or self.D is None:
                return False
            tgt = self.map_target(hs[:2], hs[2])
            if tgt is None:
                return False
            x, y, th = self.D
            d = math.dist((x, y), tgt)
            if d < 1.0:
                return False
            L = max(2.0, min(4.0, 4.0 * 5.0 / (5.0 + hs[2]), d))
            brg = math.degrees(math.atan2(tgt[1] - y, tgt[0] - x))
            end = (x + L * math.cos(math.radians(brg)), y + L * math.sin(math.radians(brg)))
            if self.S.problems((x, y), end, buf=hs[2], edge_buf=hs[2]):
                return False
            turn = (brg - th + 180.0) % 360.0 - 180.0
            if abs(turn) > 3.0:
                self.odo_step(self.rb.leg("turn", turn))
            self.odo_step(self.rb.leg("drive", L))
            self.ev("hold_creep", L=round(L, 1), to=[round(tgt[0], 1), round(tgt[1], 1)])
            self.holds_creeps = getattr(self, "holds_creeps", 0) + 1
            return True
        except Exception:  # noqa: BLE001
            return False

    def hold_never_give_up(self):
        """(e13) Held where it stopped, still and safe, with no time limit (standing still, its risk does not grow):
        NEEDS A HAND said loudly at once and every HOLD_SAY_S (terminal, status, events); the roof tag seen again ->
        carry on; the bridge's motion detector (or lock-on) sighting it -> its position is known again: the recovery is
        tried once more from there, towards where the tag read well. Only Stop (or Ctrl+C) ends it."""
        if getattr(self, "_ending", False) and PARK_GIVEUP:
            # (MIRLT_PARK_GIVEUP=1, e13's way) parking after the run ended: the run's end (report, upload, sleep) is not held up; it stays where it is
            raise RunEnd("the roof tag is not read while parking: the robot stays where it is")
        t0 = time.time()
        said = -1e9
        tried_from = None
        wig_t, wig_wait, wig_n = time.time(), 3.0, 0     # (11-10h) first find-me wiggle 3 s in, then less often
        import safety_group as SG_
        never_still = SG_.flag("MIRLT_NEVER_STILL")     # (e18) never still: a wiggle or a creep every derived pause
        if never_still:
            try:
                wig_wait = max(1.0, float(fix_interval(self.eye, self.tag)) + 0.5)
            except Exception:  # noqa: BLE001
                wig_wait = 1.0
        self.ev("hold", why="roof tag lost while driving; held with no time limit (e13)")
        while not self.stop_flag.is_set():
            if hasattr(self.eye, "is_dark") and self.eye.is_dark():
                self.wait_light("held where it stopped")     # (e14) a dark picture: no wiggles, no hand, just wait
                continue
            if self.eye.age(self.tag) < 0.3:
                d = time.time() - t0
                self.holds.append(d)
                say("  seen again after %.1f s: carrying on" % d)
                if d > 5.0:
                    self.ev("hand_rejoined", held_s=round(d, 1))
                return d
            lf = self.eye.loose_last(self.tag) if hasattr(self.eye, "loose_last") else None
            if lf is not None and time.time() - lf[0] < 0.5 and self.a.lost_policy == "recover" and \
                    (tried_from is None or math.dist(tried_from, lf[1:3]) > 3.0) and \
                    getattr(self, "_hold_depth", 0) == 0:
                tried_from = lf[1:3]
                say("  the motion detector sees it at %.0f, %.0f (no heading): trying the way back from there" %
                    lf[1:3])
                self.ev("hold_motion_seen", x=round(lf[1], 1), y=round(lf[2], 1))
                th = (self.D or (0, 0, 0))[2]
                self._hold_depth = 1
                try:
                    self.recover((lf[1], lf[2], th), 0.0, 0.0, "seen by the motion detector while held")
                finally:
                    self._hold_depth = 0
                continue
            # (11-10h, Nick 14:32: "motion should be used for finding and getting it back, not moving it by hand") a
            # robot standing still is invisible to the motion detector: it makes a small find-me wiggle on the spot
            # (only when its whole turning circle plus its position's possible error is clear of every edge, outlet and
            # keep-out), so the motion detector can see it and the recovery above can bring it back
            if time.time() - wig_t >= wig_wait and getattr(self, "_hold_depth", 0) == 0:
                hs = getattr(self, "_hold_spot", None)
                if hs is not None and not self.S.problems(hs[:2], hs[:2], buf=hs[2], edge_buf=hs[2]):
                    wig_n += 1
                    if wig_n == 1 or wig_n % 5 == 0:
                        say("  find-me wiggle %d: so the motion detector can see %s" % (
                            wig_n, getattr(self, "name", "the robot")))
                    self.ev("hold_find_me", n=wig_n)
                    t_w = time.time()
                    seen_by_pics = self.wiggle_seen()     # (e14) the wiggle watched in every camera's pictures
                    if seen_by_pics is None:
                        for d in (RECOVER_WIGGLE_DEG, -2 * RECOVER_WIGGLE_DEG, RECOVER_WIGGLE_DEG):
                            r = self.rb.leg("turn", d)
                            self.odo_step(r)
                    tried_from = None if wig_n % 3 == 0 else tried_from   # a fresh try now and then
                    lf2 = self.eye.loose_last(self.tag) if hasattr(self.eye, "loose_last") else None
                    if seen_by_pics and seen_by_pics is not True:
                        lf2 = (time.time(), seen_by_pics[0], seen_by_pics[1], "pictures")
                    if lf2 is not None and lf2[0] >= t_w and self.a.lost_policy == "recover":
                        tried_from = lf2[1:3]
                        say("  the motion detector saw the wiggle at %.0f, %.0f: driving back from there" % lf2[1:3])
                        self.ev("hold_motion_seen", x=round(lf2[1], 1), y=round(lf2[2], 1), by="wiggle")
                        th = (self.D or (0, 0, 0))[2]
                        self._hold_depth = 1
                        try:
                            self.recover((lf2[1], lf2[2], th), 0.0, 0.0, "seen by the motion detector (find-me wiggle)")
                        finally:
                            self._hold_depth = 0
                        wig_t = time.time()
                        continue
                    if not never_still:
                        wig_wait = min(HOLD_SAY_S, wig_wait * 2)     # less often while nothing sees it
                    elif wig_n % 2 == 0:
                        self.hold_creep(hs)                         # (e18) and a small creep toward readable ground
                wig_t = time.time()
            if time.time() - said >= HOLD_SAY_S:
                said = time.time()
                w = ("%s NEEDS A HAND: the camera lost its roof tag %.0f s ago and it is held where it stopped; it "
                     "wiggles now and then so the motion detector can find it and bring it back. If nothing sees it, "
                     "move it a hand's width back towards the middle of zones 1-2 (or wipe / turn its tag to the "
                     "camera): it carries on by itself once the tag is read. Stop ends the calibration.%s" % (
                         getattr(self, "name", "the robot"), time.time() - t0,
                         (" " + self.light_words() + ".") if self.light_words() else ""))
                say("!! " + w)
                self.set_status(words=w)
                self.ev("needs_a_hand", held_s=round(time.time() - t0, 1))
            time.sleep(0.1)
        raise RunEnd("stopped (Ctrl+C or Mission Control's Stop) while the robot was held waiting for a hand")

    # ------------------------------------------------------------------------------------------------- recovery (11-05c)
    def bubble(self, blind_cm, blind_deg, after_cm=None):
        """The dead-reckoning bubble (cm): base + per cm driven blind + the sideways effect of the heading error of
        the degrees turned blind over the distance driven after them (after_cm; default all of it)."""
        uncal = self.safe_frac() > SAFE_FRAC
        per = BLIND_PER_CM_UNCAL if uncal else BLIND_PER_CM
        tf = BLIND_TURN_FRAC_UNCAL if uncal else BLIND_TURN_FRAC
        after = blind_cm if after_cm is None else after_cm
        return BLIND_BASE_CM + per * blind_cm + after * math.sin(math.radians(min(60.0, tf * blind_deg)))

    def map_target(self, at, bub=0.0):
        """(e14) The nearest cell middle the readability map (this tag's where it has evidence, else the run's) calls
        readable, at least a cell away from where the tag was lost, whose middle is clear of the edges and keep-outs
        with the error sum; None if there is none."""
        try:
            rm = self.readmap()
            rt = self.readmap_tag()
        except Exception:  # noqa: BLE001
            return None
        lost = getattr(self, "last_lost_at", None) or at
        best = None
        for i in range(rm.nx):
            for j in range(rm.ny):
                c = rm.centre(i, j)
                m = rt if rt.tries.get((i, j), 0) >= 3 else rm
                if not m.readable(*c)[0] or math.dist(c, lost) < rm.cell:
                    continue
                if self.S.problems(c, c, buf=bub, edge_buf=bub):
                    continue
                d = math.dist(c, at)
                if best is None or d < best[0]:
                    best = (d, c)
        return None if best is None else best[1]

    def lost_estimate(self, cam0, r, kind, speed):
        """Where the robot is after the tag was lost in a leg, and how far / how much it moved blind: the last camera
        read (its turning point) moved on along the leg by what it drove since (this leg's speed x the time since that
        read, at most the leg's own length); the heading from the leg's own record. The bubble grows only from the
        last read, not from the leg's start."""
        end = compose(cam0, relative(r["start"], r["end"])) if (r.get("start") and r.get("end")) else cam0
        leg_cm = math.hypot(end[0] - cam0[0], end[1] - cam0[1])
        last = getattr(self, "_lost_last", None)
        if last is None or kind != "go":
            turned = abs(wrap180(end[2] - cam0[2])) if kind == "turn" else 0.0
            return end, leg_cm, turned
        t, x, y, th = last
        c, s_ = math.cos(math.radians(th)), math.sin(math.radians(th))
        ox, oy = self.offset
        px, py = x - (c * ox - s_ * oy), y - (s_ * ox + c * oy)
        h = math.radians(end[2])
        # what it drove after the last read: the leg's own end (its wheels, from the leg's start) less where the camera
        # last had it, along the heading; its error is the leg's whole length's (the allowance's share: extra_cm)
        blind = max(0.0, min(leg_cm, (end[0] - px) * math.cos(h) + (end[1] - py) * math.sin(h)))
        self._lost_extra = self.safe_frac() * leg_cm
        return (px + blind * math.cos(h), py + blind * math.sin(h), end[2]), blind, 0.0

    def recover(self, est, blind_cm, blind_deg, why):
        """(Nick, 5 Nov 16:30) The roof tag was lost while driving (the leg was stopped): dead-reckon slowly (turn on
        the spot, then RECOVER_STEP_CM straight steps at FINAL_CMS) towards the nearest spot where the tag read
        reliably, each step checked so the whole bubble (BLIND_* : larger while uncalibrated) stays clear of the edges,
        the camera's view edge and the outlets / tiles / parked robots; seen again -> the odometry-vs-camera jump is
        logged and the run resumes. A margin reached or the bubble over RECOVER_BUBBLE_CAP -> stop there, a small
        find-me wiggle, then the old hold (last resort). Returns the seconds it took."""
        t0 = time.time()
        x, y, th = est
        self.last_lost_at = (x, y)                 # (e14) where the tag was lost: evidence for a deferred point
        rec = dict(t0=round(t0, 2), why=why, start=[round(x, 1), round(y, 1)], uncalibrated=self.safe_frac() > SAFE_FRAC,
                   steps=0, blind_cm=round(blind_cm, 1))
        say("RECOVER: the roof tag was lost (%s): dead reckoning slowly towards where it last read well" % why)
        self.set_status(words="RECOVERING: the camera lost the roof tag; driving slowly back towards where it read well")
        self.ev("recover_start", **rec)
        self.rb.set("e02_cms", FINAL_CMS)

        def seen():
            return self.eye.age(self.tag) < 0.3

        def clear(a, b, bub):
            # the whole body's circle (6.72 cm) + the bubble from the edges / view / keep-outs: Site.problems keeps the
            # middle edge_in from the edge; the bubble is the leg's possible error
            return not self.S.problems(a, b, buf=bub, edge_buf=bub)
        outcome = None
        lat0 = self.bubble(blind_cm, blind_deg) - self.bubble(blind_cm, 0.0)   # the interrupted leg's own heading part
        lat0 += getattr(self, "_lost_extra", 0.0)        # the leg's length error behind the estimate (lost_estimate)
        self._lost_extra = 0.0
        after = 0.0                              # cm driven since the recovery's own turns

        def bub_now(extra=0.0):
            return self.bubble(blind_cm + extra, blind_deg, after + extra) + lat0
        while not seen() and not self.stop_flag.is_set():
            if self.wait_light("its recovery"):              # (e14) never drive blind because the picture went dark
                continue
            bub = bub_now()
            spots = sorted(self.reliable, key=lambda q: math.dist(q, (x, y)))
            tgt = next((q for q in spots if math.dist(q, (x, y)) > 1.0), None)
            if tgt is None:
                # (e14, school 16:22: "recovery stopped (no reliable spot known)") no spot where the tag read well in
                # this run yet: the nearest spot the readability map (this tag's, at today's light) says reads,
                # clear of the edges and keep-outs; the motion detector's sightings on the way come through the hold
                tgt = self.map_target((x, y), bub)
                if tgt is not None and not rec.get("map_target"):
                    rec["map_target"] = [round(tgt[0], 1), round(tgt[1], 1)]
                    say("  no spot where the tag read well yet: heading for %.0f, %.0f, where the readability map "
                        "says tags read" % tgt)
            if tgt is None:
                outcome = "no reliable spot known, and the readability map has no readable spot it can reach"
                break
            want = wrap180(math.degrees(math.atan2(tgt[1] - y, tgt[0] - x)) - th)
            if abs(want) > 3.0:
                if not clear((x, y), (x, y), bub):      # as every turn of the calibration: the margin holds the body
                    outcome = "a turn here would bring the bubble to a margin"
                    break
                r = self.rb.leg("turn", max(-MAX_TURN_DEG, min(MAX_TURN_DEG, want)), on_tick=self.stop_flag.is_set)
                self.odo_step(r)
                if r.get("start") and r.get("end"):
                    d = wrap180(r["end"][2] - r["start"][2])
                    th = wrap180(th + d)
                    blind_deg += abs(d)
                continue
            step = min(RECOVER_STEP_CM, math.dist(tgt, (x, y)))
            nx, ny = x + step * math.cos(math.radians(th)), y + step * math.sin(math.radians(th))
            bub2 = bub_now(step)
            if BLIND_RULE == "cap" and (bub2 > RECOVER_BUBBLE_CAP or blind_cm + step > RECOVER_MAX_CM):
                outcome = "the bubble would pass %.0f cm" % RECOVER_BUBBLE_CAP
                break
            stop_ = 0.0
            if BLIND_RULE != "cap":
                try:
                    import safety_rules as SR_
                    stop_ = float(SR_.stop_cm(FINAL_CMS))      # (e13) its stop at the step's speed, measured
                except Exception:  # noqa: BLE001
                    stop_ = 1.4
            if not clear((x, y), (nx, ny), bub2 + stop_):
                outcome = ("the next step's possible error (bubble %.1f cm + its stop %.1f cm) could reach the edge "
                           "margin" % (bub2, stop_))
                break
            r = self.rb.leg("go", step, on_tick=lambda: seen() or self.stop_flag.is_set())
            self.odo_step(r)
            rec["steps"] += 1
            if r.get("start") and r.get("end"):
                rel = relative(r["start"], r["end"])
                x, y, th = compose((x, y, th), rel)
                blind_cm += math.hypot(rel[0], rel[1])
                after += math.hypot(rel[0], rel[1])
            if r["result"] in ("refused", "timeout"):
                outcome = "the robot %s a recovery step" % r["result"]
                break
        if self.stop_flag.is_set():
            raise RunEnd("stopped (Ctrl+C or Mission Control's Stop)")
        if seen():
            cam = self.cam_pose(0.6)
            if cam is None:     # (11-05d) eye.last is (t, x, y, th) and raw: never its time as a position
                l = self.eye.last(self.tag)
                cam = (l[1], l[2], l[3]) if l else None
            jump = math.dist(cam[:2], (x, y)) if cam else float("nan")
            rec.update(outcome="seen again", jump_cm=round(jump, 2), bubble_cm=round(bub_now(), 2),
                       blind_cm=round(blind_cm, 1), within=bool(jump <= bub_now()),
                       dur_s=round(time.time() - t0, 1))
            say("  seen again after %.1f s, %.0f cm blind: odometry was %.1f cm off the camera (bubble %.1f cm)" % (
                rec["dur_s"], blind_cm, jump, rec["bubble_cm"]))
        else:
            rec.update(outcome="stopped: %s" % outcome, blind_cm=round(blind_cm, 1))
            say("  recovery stopped (%s): a small find-me wiggle, then the old hold" % outcome)
            bub = bub_now()
            self._hold_spot = (x, y, bub)          # (11-10h) the hold's find-me wiggles use the same clearance check
            if clear((x, y), (x, y), bub):
                for d in (RECOVER_WIGGLE_DEG, -2 * RECOVER_WIGGLE_DEG, RECOVER_WIGGLE_DEG):
                    if seen():
                        break
                    r = self.rb.leg("turn", d)
                    self.odo_step(r)
                rec["wiggle"] = True
            if not seen():
                rec["held_s"] = round(self.hold_until_seen(), 1)
            rec["dur_s"] = round(time.time() - t0, 1)
        self.ev("recover_end", **rec)
        self.recoveries.append(rec)
        self._recovered = True
        return time.time() - t0

    def drive_leg(self, kind, value, label, phase, tour=0, corrected=False, cam0=None):
        """One leg, measured: commanded `kind value`, the robot's own record, the camera before and after."""
        if self.wait_light("before its next leg"):           # (e14) no leg while the picture is dark
            cam0 = self.cam_pose_sure()                      # where the camera has it now the picture is back
        if cam0 is None:
            cam0 = self.cam_pose()
            if cam0 is None:
                self.hold_until_seen()
                cam0 = self.cam_pose()
        self._leg_moving = True
        self._lost_stop = False
        t0 = time.time()
        held = 0.0
        speed = None
        send = value
        if kind == "go":
            speed = FINAL_CMS if label.endswith("final") else SPEED_CMS
            self.rb.set("e02_cms", speed)
            send = lead_value(value, speed, getattr(self, "lead", None))
        self._sent, self._speed = send, speed
        r = self.rb.leg(kind, send, on_tick=self.watch)
        if r["result"] == "refused" and "compass" in (r.get("info") or ""):
            # (11-05h, Observe then trust: robot state) it restarted mid-run (settings and compass check gone): the
            # compass check and the safety layers again, then the leg (it never started, so it is not a repeat)
            say("  %s restarted (it refused the leg: %s): compass check and safety layers again" % (self.name,
                                                                                                r.get("info")))
            self.ev("restarted", robot=self.name, info=r.get("info"))
            self.ensure_compass(self.rb)
            ok_, w_ = arm_safety(self.rb, self.a.safe_edge)
            if not ok_:
                raise RunEnd("%s restarted and its safety layers did not read back (%s): stopped" % (self.name, w_))
            for k_ in PARAMS:                      # its values for this phase (a restart puts the stored ones back)
                self.rb.set_checked(k_, (getattr(self, "fit", None) or {}).get("new", self.phase_a_values).get(
                    k_, self.phase_a_values[k_]))
            r = self.rb.leg(kind, send, on_tick=self.watch)
        self._leg_moving = False
        self.odo_step(r)
        hz = (getattr(self.link, "tilt_hazard", {}) or {}).pop(self.rb.name, None)
        if hz:
            # (11-05h, protection 2) a standing tilt under the robot's own 5 deg stop: the same as a tilt trip
            r = dict(r, result="failed", info="standing tilt %.1f deg for %.1f s (tilted: a wheel over an edge?)" % (
                hz["dev_deg"], hz["for_s"]))
        self.after_trip(r, cam0)
        interrupted = bool(self._lost_stop)
        if self._lost_stop and self.a.lost_policy == "recover":
            # (11-05c) where it is now: the camera's pose at the leg's start moved by the robot's own record of the leg
            est, blind_cm, blind_deg = self.lost_estimate(cam0, r, kind, speed)
            held = self.recover(est, blind_cm, blind_deg, "%s %.0f %s leg" % (kind, value, label))
        elif self._lost_stop:
            held = self.hold_until_seen()
        if self.stop_flag.is_set():
            raise RunEnd("stopped (Ctrl+C or Mission Control's Stop)")
        time.sleep(SETTLE_S)
        cam1 = self.cam_pose()
        if cam1 is None and self.a.lost_policy == "recover" and not interrupted:
            # (11-05c) the tag gone at the leg's end (a short leg ends inside LOST_HOLD_S): the same recovery
            self._lost_last = self.eye.last(self.tag)
            est, bc, bd = self.lost_estimate(cam0, r, kind, speed)
            interrupted = self._lost_stop = True
            held += self.recover(est, bc, bd, "the tag gone after the %s %.0f %s leg" % (kind, value, label))
            cam1 = self.cam_pose()
        tries = 0
        while cam1 is None:
            # (11-05d, school 18:33) one read ends the hold but the pose needs 3 in MEASURE_S: hold again.
            # (11-10g, Nick 13:34: "if it finds it and doesn't move it, can't it trust that it knows where it is?") the
            # robot is standing still, so its position cannot drift: measure over a longer window each time instead of
            # ending the run (dim light: ~7 pictures a second and only some reads)
            tries += 1
            if tries > 6 and __import__("safety_group").flag("MIRLT_CALIB_GIVEUP"):
                raise RunEnd("the camera reads the roof tag only now and then after the %s %.0f leg: the robot "
                             "stopped where it is (more light on the table helps)" % (kind, value))
            if tries == 7:
                # (e13, school 12:45: the room lights went out, tag reads became intermittent and the run ended) a dim
                # room is a reason to stand still safely and ask for light, never to end the run
                lw = self.light_words()
                say("!! %s: the camera reads its roof tag only now and then after the %s %.0f leg%s. It holds, still "
                    "and safe, and carries on once the tag reads steadily." % (
                        getattr(self, "name", "the robot"), kind, value, (": " + lw) if lw else ""))
                self.ev("intermittent_reads", leg="%s %.0f" % (kind, value), light=lw)
            dur = min(STILL_MEASURE_MAX_S, MEASURE_S * (2 ** tries))
            if tries in (1, 3) or tries % 5 == 0:
                say("  the robot stands still: the camera reads its tag only now and then, so it is measured over %.0f s"
                    " (the robot cannot drift while it stands)" % dur)
            if self.stop_flag.is_set():
                raise RunEnd("stopped (Ctrl+C or Mission Control's Stop)")
            if self.eye.age(self.tag) > 5.0:
                held += self.hold_until_seen()
            cam1 = self.cam_pose(dur)
        if cam0 is None:
            interrupted = True          # (11-05d) no start pose: the leg cannot be measured, kept out of the fit
            cam0 = cam1
        t1 = time.time()
        row = self.leg_row(phase, tour, kind, label, value, r, cam0, cam1, t0, t1, corrected, held)
        if kind == "go" and r["result"] == "done" and not interrupted:
            import observe as OB
            out_ = OB.move_outcome(value, row["dt"] if row["dt"] > 0 else t1 - t0, wheel_cm=row["bel_len"],
                                   cam_moved_cm=row["cam_len"], cam_fresh=True, robot_info=r.get("info") or "")
            if out_["verdict"] != "moved":
                # (11-05h, protection 3) reported done far too fast with no camera motion: NOT done; out of the fit
                row["interrupted"] = True
                self.ev("done_without_moving", n=self.leg_n, label=label, value=value, took_s=row["dt"],
                        cam_cm=round(row["cam_len"], 2))
                say("  %s: the %.0f cm leg was reported done in %.2f s: %s: NOT done (left out of the fit; the next "
                    "leg is aimed from the camera, never the same leg unchanged)" % (self.name, value, row["dt"],
                                                                                     out_["words"]))
        if kind == "go" and not interrupted and r["result"] == "done":
            self.watch_heading(row, value)
        try:
            # (11-05h) where it was blind (the estimate) counts as a miss; where the leg ended seen, as a read
            for rm_ in (self.readmap(), self.readmap_tag()):    # (e13) the site's map and this tag's own
                if interrupted and kind == "go" and cam0 is not None:
                    hx = math.radians(cam0[2])                  # where the leg was taking it (it went blind there)
                    for _ in range(3):
                        rm_.add(cam0[0] + value * math.cos(hx), cam0[1] + value * math.sin(hx), False)
                rm_.add(cam1[0], cam1[1], True)
        except Exception:  # noqa: BLE001
            pass
        if interrupted:
            # (11-05c) a leg the camera lost and the robot was stopped in (then recovered or held): not in the fit
            row["interrupted"] = True
            self.ev("leg_interrupted", n=self.leg_n, phase=phase, label=label, policy=self.a.lost_policy)
        if r["result"] == "timeout" and kind == "turn":
            # (e13, the 10 Nov 11:54 school calibration: "turn -90" timed out; the camera saw -98.4 deg; it hunted ~40 s)
            ct = wrap180(cam1[2] - cam0[2]) if (cam0 is not None and cam1 is not None) else None
            dg = r.get("diag") or {}
            w = ("TURN DID NOT SETTLE: asked %.0f deg; the camera saw it turn %s; its own heading (gyro) said %s; %s" % (
                value, "%.1f deg" % ct if ct is not None else "nothing (tag not read)",
                "%.1f deg" % dg["own_th"] if dg.get("own_th") is not None else "nothing (no pose came back)",
                dg.get("words") or "no end report"))
            self.ev("turn_not_settled", asked=value, camera_deg=ct, own_th=dg.get("own_th"), diag=dg)
            raise RunEnd(w)
        if r["result"] in ("refused", "timeout"):
            dg = (r.get("diag") or {}).get("words")
            raise RunEnd("the robot %s the %s %.0f leg (%s)%s: stopped" % (r["result"], kind, value, r["info"],
                                                                           ("; " + dg) if dg else ""))
        if interrupted and self.a.lost_policy == "recover" and getattr(self, "_goto_depth", 0) > 0:
            raise Recovered(cam1)
        return cam1, r, row

    def leg_row(self, phase, tour, kind, label, value, r, c0, c1, t0, t1, corrected, held):
        self.leg_n += 1
        b0, b1 = r["start"], r["end"]
        no_pose = b0 is None or b1 is None
        start_missing = bool(r.get("start_missing"))    # (e16) its own motion is not known: not a measurement
        if no_pose:
            b0 = b1 = (0, 0, 0, 0)             # (e13) written as blanks below: no pose came back, not a pose of 0
        dt_ = (b1[3] - b0[3]) / 1000.0
        cam_len = math.hypot(c1[0] - c0[0], c1[1] - c0[1])
        bel_len = math.hypot(b1[0] - b0[0], b1[1] - b0[1])
        cam_turn = wrap180(c1[2] - c0[2])
        bel_turn = wrap180(b1[2] - b0[2])
        along = cross = terr = miss = ""
        if kind == "go":
            h = math.radians(c0[2])
            ex, ey = c0[0] + value * math.cos(h), c0[1] + value * math.sin(h)
            dx, dy = c1[0] - ex, c1[1] - ey
            along = dx * math.cos(h) + dy * math.sin(h)
            cross = -dx * math.sin(h) + dy * math.cos(h)
            miss = math.hypot(dx, dy)
        elif kind == "turn":
            terr = wrap180(c1[2] - (c0[2] + value))      # the end heading's error (unambiguous, +-180 turns too)
        row = [phase, tour, self.leg_n, kind, label, "%.2f" % value, r["result"], r["info"], "%.3f" % t0, "%.3f" % t1,
               "%.2f" % c0[0], "%.2f" % c0[1], "%.2f" % c0[2], "%.2f" % c1[0], "%.2f" % c1[1], "%.2f" % c1[2],
               *(["", "", "", "", "", ""] if no_pose else
                 ["%.2f" % b0[0], "%.2f" % b0[1], "%.2f" % b0[2], "%.2f" % b1[0], "%.2f" % b1[1], "%.2f" % b1[2]]),
               "%.3f" % dt_, "%.2f" % cam_len, "%.2f" % bel_len, "%.2f" % cam_turn, "%.2f" % bel_turn,
               _f(along), _f(cross), _f(terr), _f(miss), int(bool(corrected)), "%.1f" % held,
               "%.1f" % self.S.edge_dist(c1[:2]), "%.1f" % self.S.keepout_dist(c1[:2])[0],
               int(bool(getattr(self, "_lost_stop", False))),     # (11-05c) the tag was lost in it: not in the fit
               "%.2f" % getattr(self, "_sent", value), _f(getattr(self, "_speed", None))]   # (11-05h) what was sent
        self.legs.writerow(row)
        self.legs_f.flush()
        d = dict(phase=phase, tour=tour, kind=kind, label=label, value=value, result=r["result"], dt=dt_,
                 cam0=c0, cam1=c1, bel0=b0, bel1=b1, cam_len=cam_len, bel_len=bel_len, cam_turn=cam_turn,
                 bel_turn=bel_turn, along=along, cross=cross, turn_err=terr, miss=miss, corrected=corrected,
                 speed=getattr(self, "_speed", None), t0=t0, interrupted=bool(getattr(self, "_lost_stop", False)),
                 bel_ok=not (no_pose or start_missing))
        if start_missing:
            self.ev("leg_start_missing", n=self.leg_n, kind=kind, label=label,
                    words="the robot's 'started' report for this leg never came: its own motion is unknown, so the leg "
                          "is left out of the fit")
        self.rows.append(d)
        return d

    # ------------------------------------------------------------------------------------------------- moving
    def goto(self, lab, phase, tour, cam=None, odometry=True, depth=0, name=None):
        """(11-05c) _goto, aimed again from the camera after a lost-tag recovery (at most 4 times). (e14, the simulator
        rehearsal 10 Nov: P04 tried four times in a row) a tour point whose tag was just lost on the way is NOT aimed at
        again at once: Deferred (the tour moves it to its end, then skips it if it fails again); parking and detours
        keep the old aim-again (MIRLT_RETRY_LOST=again: the old way for tour points too)."""
        self._goto_depth = getattr(self, "_goto_depth", 0) + 1
        try:
            for k in range(4):
                try:
                    return self._goto(lab, phase, tour, cam=cam, odometry=odometry, depth=depth, name=name)
                except Recovered as e:
                    if self._goto_depth > 1:
                        raise                      # the outermost goto aims again
                    if RETRY_LOST != "again" and isinstance(lab, str) and lab in self.S.points and \
                            lab != self.S.park_label and getattr(self, "_in_tour", False):
                        raise Deferred(e.cam, "the camera lost the roof tag on the way to %s (near %s)" % (
                            lab, "%.0f, %.0f" % tuple(self.last_lost_at) if getattr(self, "last_lost_at", None)
                            else "where it was"))
                    say("  after the recovery: aiming at %s again from where the camera sees it" % (name or lab))
                    cam, odometry = e.cam, False
            if lab in self.S.points and lab != self.S.park_label:
                # (11-05h, Observe then trust: an unreadable area narrows the run, it does not end it) the camera loses
                # the tag on the way to this point every time: the point is left out of this robot's tours
                raise SkipPoint(cam, "the camera lost the roof tag on the way to %s four times (it does not read tags "
                                     "there)" % lab)
            raise RunEnd("lost the roof tag on the way to %s four times: stopped" % (name or lab))
        finally:
            self._goto_depth -= 1

    def _goto(self, lab, phase, tour, cam=None, odometry=True, depth=0, name=None):
        """Drive to point lab: from the robot's own pose (odometry) unless safety needs the camera's first. Returns the
        camera pose at the end."""
        tx, ty = self.S.aim(lab) if (isinstance(lab, str) and lab in self.S.points) else lab
        nm = name or str(lab)
        cam = cam or self.cam_pose_sure()          # (e13) never None (11-10c's crash, elsewhere)
        corrected = False
        d = self.D if (odometry and self.D is not None) else cam
        near = isinstance(lab, str) and lab in self.S.points and self.S.near_edge(lab)
        if depth == 0:
            blocked = [q for q in self.S.problems(cam[:2], (tx, ty)) if solid(q)]
            if blocked:
                w = self.S.via(cam[:2], (tx, ty))
                if w is not None:
                    say("  %s: round %s by (%.0f, %.0f)" % (lab, blocked[0], w[0], w[1]))
                    self.ev("detour", point=lab, via=w, why=blocked[0])
                    cam = self.goto(w, phase, tour, cam=cam, odometry=odometry, depth=1, name="%s via" % lab)
                    # (5 Nov) from where the detour ended: with odometry off, d was still the pose BEFORE the detour
                    # (the leg was aimed from the wrong place: found staging a robot to P05 round the zone 1 plate)
                    d = self.D if (odometry and self.D is not None) else cam
        for attempt in range(2):
            dist = math.hypot(tx - d[0], ty - d[1])
            turn = wrap180(math.degrees(math.atan2(ty - d[1], tx - d[0])) - d[2])
            go = dist - (NEAR_EDGE_STAGE_CM if near and dist > NEAR_EDGE_STAGE_CM + 2 else 0.0)
            # where it would really go, from where the camera sees it (+ the leg's possible error while on odometry)
            h = math.radians(cam[2] + turn)
            end = (cam[0] + go * math.cos(h), cam[1] + go * math.sin(h))
            # (5 Nov 11:00) the leg's own error: a robot still uncalibrated (phase A from the defaults, or staging) can
            # be 7 % off in length and 6 deg in heading: SAFE_FRAC_UNCAL of the leg, not 4 %; a leg aimed from the
            # camera still runs on the robot's own wheels: its length error counts too
            fr = self.safe_frac()
            buf = (SAFE_BUF_CM + SAFE_FRAC * go) if (odometry and not corrected) else 0.0
            ebuf = (SAFE_BUF_CM + fr * go) if (odometry and not corrected) else (fr - SAFE_FRAC) * go
            prob = self.S.problems(cam[:2], end, buf=buf, edge_buf=ebuf)
            if prob and attempt == 0 and odometry:
                say("  safety: from where the camera sees it, the leg to %s would reach %s: pose corrected first"
                    % (lab, prob[0]))
                self.reanchor(cam, "leg to %s would reach %s" % (lab, prob[0]))
                d = cam
                corrected = True
                continue
            if prob and depth < 2 and solid(prob[0]):
                w = self.S.via(cam[:2], (tx, ty))       # (rehearsal, 4 Nov) arrived off the last detour: one more
                if w is not None:
                    say("  %s: round %s by (%.0f, %.0f), from where the camera sees it" % (lab, prob[0], w[0], w[1]))
                    self.ev("detour", point=lab, via=w, why=prob[0])
                    cam = self.goto(w, phase, tour, cam=cam, odometry=False, depth=2, name="%s via" % nm)
                    return self.goto(lab, phase, tour, cam=cam, odometry=False, depth=2, name=name)
            if prob:
                self.ev("skip_point", point=lab, why=prob[0])
                say("  %s skipped: even from the camera's pose the leg reaches %s" % (lab, prob[0]))
                return cam
            break
        if abs(turn) > MAX_TURN_DEG:
            cam, _, _ = self.drive_leg("turn", turn / 2.0, "%s turn half" % nm, phase, tour, corrected, cam0=cam)
            cam, _, _ = self.drive_leg("turn", turn / 2.0, "%s turn half" % nm, phase, tour, corrected, cam0=cam)
        elif abs(turn) > 1.0:
            cam, _, _ = self.drive_leg("turn", turn, "%s turn" % nm, phase, tour, corrected, cam0=cam)
        if go > 0.5 and abs(turn) > 1.0:
            # (5 Nov, two-robot rehearsal: a 173 deg turn 6 deg short on the uncalibrated gyro, then a 47 cm drive along
            # the edge ended 9.5 cm from it, under the 13 cm margin) the turn's own error was not in the check above:
            # where the drive would really end along the heading the camera sees now; unsafe -> a camera turn first
            h2 = math.radians(cam[2])
            end2 = (cam[0] + go * math.cos(h2), cam[1] + go * math.sin(h2))
            p2 = self.S.problems(cam[:2], end2, buf=SAFE_BUF_CM + SAFE_FRAC * go,
                                 edge_buf=SAFE_BUF_CM + self.safe_frac() * go)
            want = wrap180(math.degrees(math.atan2(ty - cam[1], tx - cam[0])) - cam[2])
            if p2 and 1.0 < abs(want) <= MAX_TURN_DEG:
                say("  safety: after the turn, the drive to %s would reach %s: heading fixed from the camera (%+.1f deg)"
                    % (nm, p2[0], want))
                self.reanchor(cam, "drive to %s after the turn would reach %s" % (nm, p2[0]))
                cam, _, _ = self.drive_leg("turn", want, "%s turn fix" % nm, phase, tour, True, cam0=cam)
                corrected = True
                go = max(0.0, math.hypot(tx - cam[0], ty - cam[1]) -
                         (NEAR_EDGE_STAGE_CM if near and dist > NEAR_EDGE_STAGE_CM + 2 else 0.0))
        if go > FINAL_CM + 1:
            n = int(math.ceil((go - FINAL_CM) / GO_PIECE_CM))
            for _ in range(n):
                cam, _, _ = self.drive_leg("go", (go - FINAL_CM) / n, "%s drive" % nm, phase, tour, corrected, cam0=cam)
            cam, _, _ = self.drive_leg("go", FINAL_CM, "%s final" % nm, phase, tour, corrected, cam0=cam)
        elif go > 0.5:
            cam, _, _ = self.drive_leg("go", go, "%s final" % nm, phase, tour, corrected, cam0=cam)
        if near and dist > NEAR_EDGE_STAGE_CM + 2:
            # the last 10 cm near an edge: from the camera's pose (safety), at the final approach speed
            dist2 = math.hypot(tx - cam[0], ty - cam[1])
            turn2 = wrap180(math.degrees(math.atan2(ty - cam[1], tx - cam[0])) - cam[2])
            if abs(turn2) > 1.0 and dist2 > 1.0 and abs(turn2) <= MAX_TURN_DEG:
                cam, _, _ = self.drive_leg("turn", turn2, "%s edge turn" % nm, phase, tour, True, cam0=cam)
            if 0.5 < dist2 <= GO_PIECE_CM:
                cam, _, _ = self.drive_leg("go", dist2, "%s edge final" % nm, phase, tour, True, cam0=cam)
        return cam

    def safe_frac(self):
        """(5 Nov 11:00) The share of a leg's length taken as its possible error in the safety checks: SAFE_FRAC once
        the robot's values are fitted (phase C) or it started from saved values, else SAFE_FRAC_UNCAL."""
        if getattr(self, "fit", None) is not None or getattr(self, "baseline", "") in ("saved", "pooled"):
            return SAFE_FRAC
        return SAFE_FRAC_UNCAL

    def park(self, phase):
        """Back to its park spot (P11 for the first robot) by the camera (safety), facing the park heading. (e14, the
        school's 16:22 and 17:02 runs: stage() parks the robot at its spot at the START with park("stage"), which set
        _ending and never cleared it, so every hold for the rest of the run, the curve phase's and Tour C1's first step
        included, said "the roof tag is not read while parking" and ended the run) _ending is set only for the run's
        own end (phase "park") and cleared when parking returns."""
        self.set_status(words="Returning to park at %s" % self.S.park_label)
        was = getattr(self, "_ending", False)
        self._ending = phase == "park"
        try:
            return self._park(phase)
        finally:
            self._ending = was

    def _park(self, phase):
        # (e13) parking after the run. (e14, school 16:22, AL-EB54 at P16: the tour's last point
        # lost the tag and parking ended the run at once, the robot left where it was) parking is still the run: a lost
        # tag holds with the never-give-up hold and the motion-based return like any leg; MIRLT_PARK_GIVEUP=1: e13's end
        cam = self.goto(self.S.park_label, phase, 0, odometry=False)
        x, y = self.S.points[self.S.park_label]
        ax, ay = self.S.aim(self.S.park_label)
        for k in range(PARK_TRIES):
            # (5 Nov 11:00 rehearsal: an uncalibrated robot, wheels 1.07, parked 7.0 cm off P01 in one leg) once more
            # from where the camera sees it, until within PARK_TOL_CM
            if math.hypot(cam[0] - ax, cam[1] - ay) <= PARK_TOL_CM:
                break
            cam = self.goto(self.S.park_label, phase, 0, cam=cam, odometry=False, name="%s again" % self.S.park_label)
        for k in range(PARK_TRIES + 1):
            t = wrap180(self.S.park_face - cam[2])
            if abs(t) <= 3:
                break
            for t_ in ((t / 2.0, t / 2.0) if abs(t) > MAX_TURN_DEG else (t,)):
                cam, _, _ = self.drive_leg("turn", t_, "park face", phase, 0, True, cam0=cam)
        miss = math.hypot(cam[0] - x, cam[1] - y)
        self.ev("parked", miss_cm=miss, pose=cam)
        say("%s parked at %s: %.1f cm from the sticker, facing %.0f deg" % (self.name, self.S.park_label, miss, cam[2]))
        return cam, miss

    def check_battery(self):
        import engine_checks as EC
        st = self.rb.status()
        if not st:
            return None
        b = st["battery"]
        self.battery.append((time.time(), b))
        self.status["battery"] = b
        floor = EC.battery_floor_used(site=self.site)[0]     # (e14) the site's / Mission Control's floor too
        if b < floor:
            raise LowBattery("battery %d %% is under the floor %d %% (engine_checks.battery_floor)" % (b, floor))
        return b

    # ------------------------------------------------------------------------------------------------- phases
    def tour(self, phase, k, order):
        say("Tour %s%d: %s" % (phase, k, " ".join(order)))
        cam = self.cam_pose()
        if cam is not None:
            self.D = tuple(cam)             # each tour starts from where the camera has it (the raw track does not)
        order = list(order)
        deferred = set()
        self._in_tour = True
        try:
            return self._tour(phase, k, order, cam, deferred)
        finally:
            self._in_tour = False

    def _tour(self, phase, k, order, cam, deferred):
        j = -1
        while j + 1 < len(order):
            j += 1
            lab = order[j]
            if self.stop_flag.is_set():
                raise RunEnd("stopped")
            self.set_status(words="Phase %s tour %d: point %s (%d of %d)" % (phase, k, lab, j + 1, len(order)),
                            point=lab)
            pk = self.S.parked_near(self.S.aim(lab))
            if pk:
                # (5 Nov) another robot is parked there: the point is left out of this robot's tours
                self.ev("skip_point", point=lab, why="%s (%.1f cm)" % pk)
                say("  %s skipped: %s stands there" % (lab, pk[0]))
                continue
            rm = self.readmap()
            ok_, why_ = rm.readable(*self.S.points[lab])
            rt_ = self.readmap_tag()
            if rt_.tries.get(rt_.ij(*self.S.points[lab]), 0) >= 3:
                # (e13: tag 2 read at P11 in the 9 Nov tours, tag 4 lost it on 10 Nov) where this tag's own evidence
                # exists it decides: readability kept per tag where it differs from the site's
                okt, whyt = rt_.readable(*self.S.points[lab])
                if okt != ok_:
                    self.ev("readability_per_tag", point=lab, tag=self.tag, site=ok_, this_tag=okt)
                    say("  %s: for tag %s the evidence differs from the site's (%s; site: %s): this tag's is used" % (
                        lab, self.tag, whyt, why_))
                ok_, why_ = okt, "tag %s: %s" % (self.tag, whyt)
                rm = rt_
            if not ok_ and rm.tries.get(rm.ij(*self.S.points[lab]), 0) >= 3:
                # (11-05h, Observe then trust) learned in this run: the camera does not read the tag there (the typed
                # view limit stays a first guess): left out of the tours, said, the run carries on
                self.ev("skip_point", point=lab, why="the camera does not read tags there (%s)" % why_)
                say("  %s skipped: the camera does not read tags there (learned in this run: %s)" % (lab, why_))
                continue
            try:
                cam = self.goto(lab, phase, k, cam=cam)
            except Deferred as e:
                # (e14) not re-tried at once: the spot where the tag was lost is evidence; the point goes to the end of
                # the tour once, and is skipped (with the reason) if the tag is lost on the way again
                lost = getattr(self, "last_lost_at", None)
                if lost is not None:
                    for hx_ in (self.readmap(), self.readmap_tag()):
                        hx_.add(lost[0], lost[1], False)
                if lab not in deferred:
                    deferred.add(lab)
                    order.append(lab)
                    self.ev("point_deferred", point=lab, why=e.why, lost_at=lost)
                    say("  %s moved to the end of this tour: %s (not tried again at once)" % (lab, e.why))
                    cam = e.cam or self.cam_pose()
                    continue
                self.ev("skip_point", point=lab, why=e.why + "; a second time, at the end of the tour")
                say("  %s skipped: %s, a second time (it was moved to the end of the tour once)" % (lab, e.why))
                self.skipped_pts = set(getattr(self, "skipped_pts", set())) | {lab}
                for hx_ in (self.readmap(), self.readmap_tag()):
                    for _ in range(3):
                        hx_.add(*self.S.points[lab], False)
                cam = e.cam or self.cam_pose()
                continue
            except SkipPoint as e:
                self.ev("skip_point", point=lab, why=e.why)
                say("  %s skipped: %s; the tour goes on" % (lab, e.why))
                self.skipped_pts = set(getattr(self, "skipped_pts", set())) | {lab}
                for hx_ in (self.readmap(), self.readmap_tag()):
                    for _ in range(3):
                        hx_.add(*self.S.points[lab], False)
                cam = e.cam or self.cam_pose()
                continue
            x, y = self.S.points[lab]
            self.ev("point", phase=phase, tour=k, point=lab, cam=cam, off_cm=math.hypot(cam[0] - x, cam[1] - y))
            time.sleep(max(0.0, DWELL_S - MEASURE_S - SETTLE_S))
            if (j + 1) % BATTERY_CHECK_EVERY == 0:
                self.check_battery()
        return cam

    def still(self, secs, phase, tour=0):
        """Gyro drift: one `turn 0` leg whose standing-still start (edge_learn_ms, the edge guard learning the table)
        is made secs long, so the robot's own heading is tracked through it (a robot with no camera fix starts each
        leg's pose afresh, so the drift between legs is not in its records). edge_learn_ms is put back after."""
        self.set_status(words="Standing still %.0f s (gyro drift)" % secs)
        old = self.rb.get("edge_learn_ms")
        old = 300.0 if old is None else old
        self.rb.set_checked("edge_learn_ms", secs * 1000.0)
        c0 = self.cam_pose()
        t0 = time.time()
        r = self.rb.leg("turn", 0.0, timeout=secs + 15.0, on_tick=lambda: self.stop_flag.is_set())
        self.rb.set_checked("edge_learn_ms", old)
        c1 = self.cam_pose()
        if r["result"] != "done" or None in (c0, c1) or r["start"] is None:
            say("  still period: no usable record (%s %s)" % (r["result"], r["info"]))
            return None
        b0, b1 = r["start"], r["end"]
        dt_ = (b1[3] - b0[3]) / 1000.0
        d = dict(phase=phase, tour=tour, kind="still", label="still", value=secs, dt=dt_, cam0=c0, cam1=c1, bel0=b0,
                 bel1=b1, bel_turn=wrap180(b1[2] - b0[2]), cam_turn=wrap180(c1[2] - c0[2]), result="done")
        self.rows.append(d)
        self.legs.writerow([phase, tour, 0, "still", "still", "%.1f" % secs, "done", "", "%.3f" % t0,
                            "%.3f" % time.time()] + ["%.2f" % v for v in (c0[0], c0[1], c0[2], c1[0], c1[1], c1[2],
                                                                           b0[0], b0[1], b0[2], b1[0], b1[1], b1[2])]
                           + ["%.3f" % dt_, "", "", "%.2f" % d["cam_turn"], "%.2f" % d["bel_turn"], "", "", "", "", 0,
                              0, "", ""])
        self.legs_f.flush()
        say("  still %.0f s: the robot's own heading moved %+.2f deg (camera %+.2f): %+.4f deg/s" % (
            dt_, d["bel_turn"], d["cam_turn"], d["bel_turn"] / max(dt_, 1e-6)))
        return d

    def phase_a(self):
        self.set_status(phase="A", words="Phase A: in-place turns at %s (%s)" % (
            self.S.park_label, "defaults 1 / 1 / 0" if getattr(self, "baseline", "") == "defaults" else "saved values"))
        say("Phase A: baseline with %s %s" % ("the defaults" if getattr(self, "baseline", "") == "defaults" else
                                              "the robot's current values", self.phase_a_values))
        cam = self.cam_pose_sure()
        rest = [self.eye.measure(self.tag)]
        nd = getattr(self, "pool_needs", None) or {"turns": True, "still": True, "rolls": True}
        for k, t in enumerate(TURNS_A if nd["turns"] else ()):
            cam, r, row = self.drive_leg("turn", float(t), "turn %+d" % t, "A", 0, cam0=cam)
            m = self.eye.measure(self.tag, 0.4)
            if m:
                rest.append(m)
        off = tag_offset([(m[0], m[1], m[2]) for m in rest if m])
        if off:
            self.offset = off[1]
            self.fit_offset = {"turning_point": off[0], "offset_cm": off[1], "rms_cm": off[2],
                               "offset_len_cm": math.hypot(*off[1])}
            say("  tag offset from the turning point: %.1f cm ahead, %.1f cm left (%.1f cm, fit +- %.1f cm)" % (
                off[1][0], off[1][1], math.hypot(*off[1]), off[2]))
            self.ev("tag_offset", **self.fit_offset)
        if not nd["turns"]:
            say("  turns: not driven (the pooled gyro values are already as sure as a finished run's; the tag offset "
                "%.1f, %.1f cm from the pool)" % tuple(self.offset))
        if nd["still"]:
            self.still(STILL_S, "A")
        else:
            say("  still period: not needed (the pooled gyro drift is already as sure as a finished run's)")
        if not nd["rolls"]:
            say("  rolls: not driven (the pooled wheel_scale is already as sure as a finished run's)")
        # rolls: towards -x from P11 (open table, away from the middle outlet and the camera's far limit)
        self.set_status(words="Phase A: straight rolls %s cm out and back" % " / ".join("%.0f" % v for v in
                                                                                   sorted(set(ROLLS_A))))
        cam = self.cam_pose_sure()
        far = max(ROLLS_A)
        hd = next((h for h in ROLL_DIRS if not self.S.problems(cam[:2], (cam[0] + far * math.cos(math.radians(h)),
                                                                       cam[1] + far * math.sin(math.radians(h))),
                                                                buf=SAFE_FRAC * far,
                                                                edge_buf=self.safe_frac() * far)), ROLL_DIRS[0])
        if hd != ROLL_DIRS[0] and nd["rolls"]:
            say("  rolls towards %+.0f deg (more room there from %s)" % (hd, self.S.park_label))
        t = wrap180(hd - cam[2]) if nd["rolls"] else 0.0
        for t_ in ((t / 2.0, t / 2.0) if abs(t) > MAX_TURN_DEG else (t,) if abs(t) > 0 else ()):
            cam, _, _ = self.drive_leg("turn", t_, "face roll", "A", 0, cam0=cam)
        for L in (ROLLS_A if nd["rolls"] else ()):
            for leg in ("out", "back"):
                end = (cam[0] + L * math.cos(math.radians(cam[2])), cam[1] + L * math.sin(math.radians(cam[2])))
                prob = self.S.problems(cam[:2], end, buf=SAFE_FRAC * L, edge_buf=self.safe_frac() * L)
                if prob:
                    say("  roll %.0f cm %s skipped: would reach %s" % (L, leg, prob[0]))
                    continue
                cam, _, _ = self.drive_leg("go", L, "roll %.0f %s" % (L, leg), "A", 0, cam0=cam)
                sg = 1.0 if leg == "out" else -1.0      # round by two quarter turns, left out, right back
                cam, _, _ = self.drive_leg("turn", 90.0 * sg, "roll turn", "A", 0, cam0=cam)
                cam, _, _ = self.drive_leg("turn", 90.0 * sg, "roll turn", "A", 0, cam0=cam)
        # back to its spot by the camera (safety), then anchor and tour
        cam = self.goto(self.S.park_label, "A", 0, odometry=False)
        self.set_status(words="Phase A: tour P01-P20 on wheels + gyro (camera for safety)")
        order = self.S.zone12()
        self.tour("A", 1, order + [self.S.park_label])

    def phase_b(self):
        self.set_status(phase="B", words="Phase B: fitting wheel_scale, gyro_scale, gyro_drift_dps")
        A = [r for r in self.rows if r["phase"] == "A" and not r.get("interrupted")   # (11-05c) interrupted legs out
             and r.get("bel_ok", True)]          # (e16) a leg whose own motion is unknown (lost "started") out
        turns = [_unwrap_turn(r) for r in A if r["kind"] == "turn" and r["result"] == "done" and abs(r["value"]) >= 30]
        still = [(r["bel_turn"], r["dt"]) for r in A if r["kind"] == "still"]
        rolls = [(r["bel_len"], r["cam_len"]) for r in A if r["kind"] == "go" and r["result"] == "done"
                 and r["value"] >= 15]
        over = [(r["bel_len"], r["cam_len"], r.get("speed") or SPEED_CMS, r["value"]) for r in A
                if r["kind"] == "go" and r["result"] == "done" and r["value"] >= 4.0 and r.get("speed")]
        fit = fit_values(turns, still, rolls, self.phase_a_values, overrun_rows=over)    # the values phase A ran with
        if POOL_ON():
            # (e14) the values from every leg pooled (this run's legs added), each with its pooled +-; saved when its +-
            # is no wider than one finished run's would be (calib_pool.verdict, derived)
            import calib_pool as CP
            fit["this_run"] = {k: fit.get(k) for k in PARAMS}
            fit["this_run_verdict"] = dict(fit.get("verdict") or {})
            pooled = CP.fit(self.pool_dir(), self.wifi_name(), extra=self.pool_rows())
            if pooled:
                for k in PARAMS:
                    if k in pooled:
                        fit[k], fit["se_" + k] = pooled[k], pooled["se_" + k]
                fit["verdict"] = CP.verdict(pooled)
                fit["pooled"] = {k: v for k, v in pooled.items() if not isinstance(v, dict)}
                fit["full_run_se"] = CP.full_run_se(pooled)
                say("  pooled with %d earlier legs: %s" % (
                    sum(pooled.get(n_, 0) for n_ in ("n_turns", "n_still", "n_rolls")) - len(self.pool_rows()),
                    CP.words(pooled, fit["full_run_se"])))
        fit["old"] = dict(self.before)
        fit["phase_a_values"] = dict(self.phase_a_values)
        fit["baseline"] = getattr(self, "baseline", "saved")
        fit["enough"] = dict(ENOUGH)
        new = dict(self.before)
        for k in PARAMS:
            if fit["verdict"].get(k) == "ok":
                new[k] = round(fit[k], 4)
        fit["new"] = new
        if fit.get("overrun_cm") and getattr(self.a, "stop_lead", "auto") != "off" and fit["verdict"].get(
                "wheel_scale") == "ok":
            self.lead = {float(k): v for k, v in fit["overrun_cm"].items() if abs(v) <= LEAD_MAX_CM}
            say("  stop lead (each leg sent this much short so it ends on the spot): %s" % ", ".join(
                "%.2f cm at %s cm/s" % (v, k) for k, v in sorted(self.lead.items())))
        fit["stop_lead_cm"] = {("%g" % k): v for k, v in (getattr(self, "lead", None) or {}).items()}
        say("Fit: " + ", ".join("%s %.4f +- %.4f (%s)" % (k, fit.get(k, float("nan")), 1.96 * fit.get("se_" + k, 0),
                                                          fit["verdict"][k]) for k in PARAMS))
        rb = {}
        for k in PARAMS:
            ok, got = self.rb.set_checked(k, new[k])
            rb[k] = {"sent": new[k], "read": got, "ok": ok}
            if not ok:
                say("  %s did NOT read back (%s): the old value is put back" % (k, got))
                self.rb.set_checked(k, self.before[k])
        fit["readback"] = rb
        fit["saved"] = self.save_on_robot(new, rb)
        self.record_calibrated(fit)
        self.fit = fit
        self.fit_t = time.time()
        with open(os.path.join(self.folder, "fit.json"), "w") as f:
            json.dump(fit, f, indent=1, default=str)
        self.ev("fit", **{k: v for k, v in fit.items() if not isinstance(v, dict)})
        with open(os.path.join(self.folder, "robot_values_after.json"), "w") as f:
            json.dump({"robot": self.name, "values": {k: rb[k]["read"] for k in PARAMS}}, f, indent=1)
        self.set_status(fit={k: rb[k] for k in PARAMS}, saved=fit["saved"])

    # ------------------------------------------------------------------------------------------------- baseline (5 Nov)
    def baseline_mode(self):
        """(Nick, 5 Nov 11:00) The FIRST calibration of a robot starts phase A from the defaults (uncalibrated
        baseline); later ones from its saved values. "First" = its WiFi name is not in this site's
        calibrated_robots.json (written when a run saved its values). --baseline-defaults / --from-saved override
        (the site's calibration.baseline: auto, defaults or saved). ("defaults" | "saved", words)."""
        mode = getattr(self.a, "baseline", "auto") or "auto"
        wifi = (getattr(self, "wifi", {}) or {}).get(self.rb.name)
        p = state_path(self.a, CALIBRATED_FILE)
        rec = load_json(p, {}) if p else {}
        if mode == "defaults":
            return "defaults", "asked for (--baseline-defaults / calibration.baseline defaults)"
        if mode == "saved":
            return "saved", "asked for (--from-saved / calibration.baseline saved)"
        if wifi and wifi in rec:
            e = rec[wifi]
            return "saved", "%s was calibrated here before (first %s, last %s): a re-calibration, from its saved values" % (
                wifi, e.get("first", "?"), e.get("last", "?"))
        return "defaults", "the first calibration of %s on this site (not in %s): an uncalibrated baseline" % (
            wifi or self.rb.name + " (WiFi name not known)", CALIBRATED_FILE)

    # ------------------------------------------------------------------------------------------------- (e14) the pool
    def pool_dir(self):
        return getattr(self.a, "state_dir", "") if self.a.dry else ((self.site or {}).get("conf_dir") or "")

    def wifi_name(self):
        return (getattr(self, "wifi", {}) or {}).get(self.rb.name) or self.rb.name

    def pool_rows(self):
        """This run's legs as the pool keeps them (raw, with the values in force and the battery then)."""
        import calib_pool as CP
        fit_t = getattr(self, "fit_t", None)
        after = (getattr(self, "fit", None) or {}).get("new") or self.phase_a_values

        def values_at(t):
            return after if (fit_t is not None and t >= fit_t) else self.phase_a_values

        def battery_at(t):
            b = [v for (tb, v) in self.battery if tb <= t]
            return b[-1] if b else (self.battery[0][1] if self.battery else None)
        # phase A's legs only, as the per-run fit uses: they are driven on wheels + gyro alone; staging, parking and the
        # tours' legs may be aimed or corrected from the camera (rehearsal: pooling them widened the wheel's +- 3x)
        rows = [r for r in getattr(self, "rows", []) if r.get("phase") == "A"]
        if self.stop_flag.is_set() and rows:
            rows = rows[:-1]       # the leg Stop cut short is not a measurement (rehearsal: it widened the wheel's +- 4x)
        return CP.raw_rows(rows,
                           values_at, battery_at, os.path.basename(self.folder.rstrip("/")), _unwrap_turn)

    def pool_after(self):
        """(e14) Keep this run's legs in the pool, whether it finished or not; when it did not reach its fit and the
        pooled values are now as sure as a finished run's, set and save them on the robot."""
        if not POOL_ON():
            return
        import calib_pool as CP
        try:
            off = getattr(self, "fit_offset", None)
            w = CP.add(self.pool_dir(), self.wifi_name(), self.pool_rows(),
                       offset=(off["offset_cm"][0], off["offset_cm"][1], off["rms_cm"]) if off else None,
                       session=os.path.basename(self.folder.rstrip("/")))
            say("POOL: %s" % w)
            self.ev("pooled", words=w)
            if getattr(self, "fit", None) is not None:
                return
            pooled = CP.fit(self.pool_dir(), self.wifi_name())
            v = CP.verdict(pooled) if pooled else {}
            say("POOL after this run (it ended before its fit): %s" % CP.words(pooled))
            if pooled and all(v.get(k) == "ok" for k in PARAMS):
                new = {k: round(pooled[k], 4) for k in PARAMS}
                rb = {}
                for k in PARAMS:
                    ok, got = self.rb.set_checked(k, new[k])
                    rb[k] = {"sent": new[k], "read": got, "ok": ok}
                sv = self.save_on_robot(new, rb)
                fit = {"new": new, "saved": sv, "baseline": "pooled", "pooled": True}
                self.record_calibrated(fit)
                self.pool_saved = True
                say("POOL: the pooled values are as sure as a finished run's: set and saved on %s (%s)" % (
                    self.name, ", ".join("%s %.4f" % kv for kv in new.items())))
                self.ev("pool_saved", values=new, saved=sv)
        except Exception as e:  # noqa: BLE001  (the pool never stops the end of a run)
            say("(calibration pool not updated: %s)" % e)

    def record_calibrated(self, fit):
        """This robot counts as calibrated once its fitted values were saved on it (save or corr save, checked)."""
        sv = fit.get("saved") or {}
        wifi = (getattr(self, "wifi", {}) or {}).get(self.rb.name)
        lp = state_path(self.a, LEADS_FILE)
        if lp and wifi and fit.get("stop_lead_cm"):
            # (11-05h) the stop lead lives on the Pi (no firmware keeps it), whatever the robot could save
            leads = load_json(lp, {})
            leads[wifi] = {"stop_lead_cm": fit["stop_lead_cm"], "wheel_scale": (fit.get("new") or {}).get(
                "wheel_scale"), "time": dt.datetime.now().isoformat(timespec="seconds"),
                "session": os.path.basename(self.folder.rstrip("/"))}
            save_json(lp, leads)
        p = state_path(self.a, CALIBRATED_FILE)
        if not (p and wifi and sv.get("ok") and sv.get("how") in ("save", "corr save")):
            return
        rec = load_json(p, {})
        now = dt.datetime.now().isoformat(timespec="seconds")
        e = dict(rec.get(wifi) or {"first": now})
        e.update(last=now, values=fit.get("new"), how=sv.get("how"), baseline=fit.get("baseline"),
                 session=os.path.basename(self.folder.rstrip("/")))
        rec[wifi] = e
        save_json(p, rec)
        say("  %s recorded as calibrated (%s): its next calibration starts from its saved values" % (wifi, p))

    # ------------------------------------------------------------------------------------------------- save (5 Nov)
    def save_on_robot(self, new, rb):
        """Nick (5 Nov): the fitted values kept on the robot over power-off. Firmware 2026-11-05a: `save`, then `saved`
        and `get` to check. Older firmware answers "unknown command": said plainly; then, unless the site's
        calibration.old_firmware_save is "off", each value with `corr save` (in the firmware since 2026-10-20b, the
        same storage), checked with `get`. Switch: calibration.save_on_robot (default on) / --no-save."""
        out = {"how": None, "ok": False, "words": ""}
        if not self.a.save_on_robot:
            out.update(how="off", words="not saved on the robot (switched off: calibration.save_on_robot / "
                                        "--no-save): the values last until it sleeps or is switched off")
            say("  " + out["words"])
            return out
        vals = {k: (rb[k]["read"] if rb[k].get("ok") else self.before[k]) for k in PARAMS}
        if all(abs(vals[k] - self.before[k]) <= PARAM_TOL for k in PARAMS):
            out.update(how="nothing new", ok=True, values=vals,
                       words="nothing new to save: the robot keeps the values it had")
            say("  " + out["words"])
            return out
        self.set_status(words="Saving the values on the robot")
        name = self.rb.name
        r = None
        for _ in range(2):
            k = self.link.mark()
            self.link.send(self.rb.ip, "save")
            r = self.link.wait_for(lambda n, l: l if n == name and (
                "saved on the robot" in l or "unknown command 'save" in l or "save refused" in l or "busy" in l)
                else None, k, 4.0)
            if r:
                break
        if r and "saved on the robot" in r:
            got = {kk: self.rb.get(kk) for kk in PARAMS}
            k = self.link.mark()
            self.link.send(self.rb.ip, "saved")
            sv = self.link.wait_for(lambda n, l: l if n == name and " calibration now:" in l else None, k, 3.0) or ""
            ok = all(got[kk] is not None and abs(got[kk] - vals[kk]) <= PARAM_TOL for kk in PARAMS) and \
                sv.count("(saved)") >= 3     # (11-05c firmware: 5 values, the arc scales too)
            out.update(how="save", ok=ok, values=got, check=sv, words=(
                "saved on the robot with `save` (firmware 2026-11-05a or newer) and checked: %s" % sv.split(": ", 1)[-1]
                if ok else "`save` answered but the check did not match (get %s; saved: %s)" % (got, sv or "no answer")))
        elif r and "unknown command" in r:
            mode = self.a.old_firmware_save
            pre = "this robot's firmware has no `save` (it needs 2026-11-05a: reflash it to get it)"
            if mode == "off":
                out.update(how="old firmware", words=pre + "; NOT saved (calibration.old_firmware_save off): the "
                                                           "values last until it sleeps or is switched off")
            else:
                for kk in PARAMS:
                    self.link.send(self.rb.ip, "corr save %s %.4f" % (kk, vals[kk]))
                    time.sleep(0.3)
                got = {kk: self.rb.get(kk) for kk in PARAMS}
                ok = all(got[kk] is not None and abs(got[kk] - vals[kk]) <= PARAM_TOL for kk in PARAMS)
                out.update(how="corr save", ok=ok, values=got, words=pre + "; saved instead with `corr save` (this "
                           "firmware has it since 2026-10-20b; the same storage, loaded at power-on)%s" % (
                               "" if ok else ": the check with get did NOT match (%s)" % got))
        else:
            out.update(how="failed", words="`save` was not answered (%s): the values are on the robot until it sleeps "
                                           "or is switched off" % (r or "nothing heard"))
        say("  SAVE: " + out["words"])
        self.ev("saved", **out)
        return out

    def phase_c(self):
        base = self.S.zone12()
        rng = random.Random(TOUR_SEED)
        shuf = list(base)
        rng.shuffle(shuf)
        orders = [("in order", base), ("reversed", base[::-1]), ("shuffled (seed %d)" % TOUR_SEED, shuf)]
        self.tours_c = []
        for k in range(min(self.a.tours, len(orders))):
            name, order = orders[k]
            self.set_status(phase="C", words="Phase C tour %d of %d (%s)" % (k + 1, self.a.tours, name))
            self.check_battery()
            self.still(STILL_C_S, "C", k + 1)
            self.tour("C", k + 1, order + [self.S.park_label])
            self.tours_c.append(name)

    # ------------------------------------------------------------------------------------------------- the curve phase (11-10b)
    def arc_wanted(self):
        """(on, words) for this robot. --arc-phase / site calibration.arc_phase "off" or "on"; "auto" (the default): on
        when its firmware drives curves (ARC_FW or newer) AND free roam here drives curves (roam_motion_for)."""
        mode = (getattr(self.a, "arc_phase", None) or "auto").lower()
        host = getattr(self, "parent", None) or self
        fw = (getattr(host, "fw", None) or {}).get(self.rb.name, "")
        if mode == "off":
            return False, "switched off (--arc-phase off, or the site's calibration.arc_phase / Mission Control's " \
                          "Advanced 'Curves in the calibration')"
        if not fw or fw < ARC_FW:
            return False, "its firmware %s cannot drive curves (they need %s or newer)" % (fw or "(not said)", ARC_FW)
        if mode == "on":
            return True, "switched on (firmware %s)" % fw
        motion, src = roam_motion_for(self.a, self.site)
        if motion != "curves":
            return False, "free roam here drives straight lines (%s), so it never curves: not needed" % src
        return True, "free roam here drives curves (%s) and its firmware %s drives them" % (src, fw)

    @staticmethod
    def arc_path(x0, y0, h0, k, L, step=ARC_SAMPLE_CM):
        """[(x, y, s)] along a constant-curvature piece from (x0, y0) heading h0 (deg), curvature k (1/cm, + = left),
        length L: the firmware's driveCurve circle (centre to the left for k > 0)."""
        n = max(2, int(math.ceil(L / step)))
        hr = math.radians(h0)
        out = []
        if abs(k) < 1e-6:
            return [(x0 + L * i / n * math.cos(hr), y0 + L * i / n * math.sin(hr), L * i / n) for i in range(n + 1)]
        R, sg = 1.0 / abs(k), (1.0 if k > 0 else -1.0)
        cx, cy = x0 - math.sin(hr) * sg * R, y0 + math.cos(hr) * sg * R
        a0 = math.atan2(y0 - cy, x0 - cx)
        for i in range(n + 1):
            s_ = L * i / n
            a = a0 + sg * s_ / R
            out.append((cx + R * math.cos(a), cy + R * math.sin(a), s_))
        return out

    def arc_room(self, x0, y0, h0, k, L, v):
        """(worst margin cm, why) for the whole arc: >= 0 = clear. Checked as if it really turned ARC_K_ALLOW tighter or
        wider and ran that much longer; at every sample its middle keeps the larger of the site's edge margin and the
        DERIVED clearance (safety_rules.clearance_cm: half diagonal + its blind bubble so far + the stop at this speed),
        the keep-out clearance (outlets, tiles, parked robots) plus the bubble, stays inside the camera's view, and
        only where the camera reads tags (this run's readability map)."""
        import safety_rules as SR
        rm = self.readmap()
        worst, why = 1e9, ""
        for kk in (k * (1 - ARC_K_ALLOW), k, k * (1 + ARC_K_ALLOW)):
            for x, y, s_ in self.arc_path(x0, y0, h0, kk, L * (1 + ARC_K_ALLOW)):
                unc = self.bubble(s_, math.degrees(abs(kk) * s_))
                need = max(self.S.edge - 0.5, SR.clearance_cm(unc, speed=v, tracking="tags"))
                e = self.S.edge_dist((x, y))
                for m, w in ((e - need, "the edge (%.1f cm from it at %.0f, %.0f; %.1f needed)" % (e, x, y, need)),
                             (self.S.keepout_dist((x, y))[0] - (KEEPOUT_CLEAR_CM + unc - 0.5),
                              "%s" % self.S.keepout_dist((x, y))[1]),
                             (self.S.view_x - (x + unc), "the edge of the camera's view"),
                             (0.0 if rm.readable(x, y)[0] else -1.0, "a spot the camera does not read (%.0f, %.0f)" % (
                                 x, y))):
                    if m < worst:
                        worst, why = m, w
        return worst, why

    def arc_centre(self):
        """The middle of the area the camera reads best: the readable spot furthest from the site's edge margin, the
        keep-outs, the view limit and every cell the camera does not read (this run's readability map). (x, y, margin)."""
        rm = self.readmap()
        bad = [rm.centre(i, j) for i in range(rm.nx) for j in range(rm.ny) if not rm.readable(*rm.centre(i, j))[0]]
        best = None
        for x in range(0, int(self.S.L) + 1, 5):
            for y in range(0, int(self.S.W) + 1, 5):
                p = (float(x), float(y))
                m = min(self.S.edge_dist(p) - self.S.edge, self.S.keepout_dist(p)[0] - KEEPOUT_CLEAR_CM,
                        self.S.view_x - x, min([math.hypot(x - b[0], y - b[1]) for b in bad] or [1e9]))
                if m > 0 and (best is None or m > best[2]):
                    best = (p[0], p[1], m)
        return best

    def arc_plan_heading(self, cam, k, L, v):
        """The start heading needing the smallest turn on the spot from where it stands that gives the arc room, or
        None: (heading, margin)."""
        cands = sorted(((wrap180(cam[2] + d), abs(d)) for d in np.arange(-180.0, 180.0, ARC_HEAD_STEP_DEG)),
                       key=lambda q: q[1])
        for h, _ in cands:
            m, _w = self.arc_room(cam[0], cam[1], h, k, L, v)
            if m >= 0:
                return h, m
        return None

    def arc_turn_to(self, cam, h):
        t = wrap180(h - cam[2])
        for t_ in ((t / 2.0, t / 2.0) if abs(t) > MAX_TURN_DEG else (t,)):
            if abs(t_) >= 2.0:
                cam, _, _ = self.drive_leg("turn", t_, "curve aim", "R", 0, True, cam0=cam)
        return cam

    def drive_arc(self, cam, k, L, v):
        """One R piece on wheels + gyro alone: `run follow`, its own pose set to the camera's (fpose), `fs <id> R ...`,
        a heartbeat (fspd 1.00) every 0.5 s, `stop` at its end (or when the camera loses the tag, it comes within 3 cm
        of the edge margin by the camera, or Stop is pressed). dict(result, why, t0, t1, own0, own1)."""
        rb, link, name = self.rb, self.link, self.rb.name
        out = {"result": None, "why": "", "t0": None, "t1": None, "own0": None, "own1": None}
        k0 = link.mark()
        link.send(rb.ip, "run follow")
        if not link.wait_for(lambda n, l: n == name and "follow: ready" in l, k0, 8.0):
            link.send(rb.ip, "stop")
            out.update(result="no follow", why="it did not start following (run follow)")
            return out
        link.send(rb.ip, "fpose %.2f %.2f %.2f 1.0 3.0" % (cam[0], cam[1], cam[2]))
        time.sleep(0.25)
        self.arc_sid = getattr(self, "arc_sid", 0) + 1
        sid = self.arc_sid
        cmd = "fs %d R %.2f %.2f %.2f %.5f %.2f %.2f -" % (sid, cam[0], cam[1], cam[2], k, L, v)
        rx = re.compile(r"^D,STEP,[^,]*,(\d+),(\w+),(\d+),(-?[0-9.]+),(-?[0-9.]+),(-?[0-9.]+),[^,]*,?(.*)$")
        fx = re.compile(r"^D,FOLLOW,[^,]*,\d+,\d+,\w+,\d+,\d+,\d+,(-?[0-9.]+),(-?[0-9.]+),(-?[0-9.]+),")
        k1 = link.mark()
        t_end = time.time() + 8.0 + L / max(v, 1.0) * 3.0
        last_send = last_hb = 0.0
        acked = False
        while time.time() < t_end:
            now = time.time()
            if not acked and now - last_send > 1.0:
                link.send(rb.ip, cmd)
                last_send = now
            if now - last_hb > 0.5:
                link.send(rb.ip, "fspd 1.00")       # the heartbeat (follow_wd_ms); scale 1 changes nothing
                last_hb = now
            r = link.wait_for(lambda n, l: (rx.match(l) or fx.match(l)) if n == name else None, k1, 0.2)
            if r and r.re is fx:
                with link.cond:
                    k1 = rb._index_after(r, k1)
                out["own_last"] = (float(r.group(1)), float(r.group(2)), float(r.group(3)))
                continue
            if r:
                with link.cond:
                    k1 = rb._index_after(r, k1)
                if int(r.group(1)) == sid:
                    st = r.group(2)
                    pose = (float(r.group(4)), float(r.group(5)), float(r.group(6)), int(r.group(3)))
                    if st in ("queued", "dup", "started"):
                        acked = True
                    if st == "started":
                        out["t0"], out["own0"] = time.time(), pose
                    elif st in ("done", "failed", "dropped", "bad", "full"):
                        out.update(result=st, why=r.group(7) or "", t1=time.time(), own1=pose)
                        if out["t0"] is None:
                            out["t0"], out["own0"] = out["t1"], pose
                        break
            if out["t0"] is not None:
                stop_why = None
                lc = self.eye.last(self.tag)
                if self.stop_flag.is_set():
                    stop_why = "stopped by hand"
                elif self.eye.age(self.tag) > LOST_HOLD_S:
                    stop_why = "the camera lost the roof tag for over %.1f s" % LOST_HOLD_S
                elif lc and self.S.edge_dist(lc[1:3]) < self.S.edge - 3.0:
                    stop_why = "the camera saw it %.1f cm from the edge (margin %.0f)" % (
                        self.S.edge_dist(lc[1:3]), self.S.edge)
                if stop_why:
                    link.send(rb.ip, "stop")
                    out.update(result="stopped", why=stop_why, t1=time.time())
                    break
        else:
            out.update(result="timeout", why="no end report", t1=time.time())
        # the follow run ends; idle legs again. (e13, the 10 Nov 11:54 school calibration: a turn leg timed out with no
        # LEG record at all) a lost `stop` left the robot following, and idle legs are not run as legs then: `stop` is
        # sent again until the robot says so (or its FOLLOW rows stop coming), at most 5 times
        for _try in range(5):
            k2 = link.mark()
            link.send(rb.ip, "stop")
            if link.wait_for(lambda n, l: n == name and ("finished" in l or "nothing running" in l), k2, 2.0):
                break
            if not link.wait_for(lambda n, l: n == name and l.startswith("D,FOLLOW"), link.mark(), 0.6):
                break
        time.sleep(0.2)
        return out

    def one_arc(self, rnd, R, sg, v, use):
        """Plan (heading, or the middle first), turn, drive, measure and fit one arc. The row (also in arcs.csv)."""
        import freeroam_core as FC
        k, L = sg / R, math.radians(ARC_SWEEP_DEG[R]) * R
        row = dict(round=rnd, use=use, radius_cm=R, side="left" if sg > 0 else "right", k=round(k, 5), L_cm=round(L, 1),
                   v_cms=round(v, 2), result="", why="", n=0, k_ratio=None, length_ratio=None, R_fit=None)
        cam = self.cam_pose_sure()                 # (e13) never None
        hp = self.arc_plan_heading(cam, k, L, v)
        if hp is None:
            c = self.arc_centre()
            if c is not None and math.hypot(c[0] - cam[0], c[1] - cam[1]) > 3.0:
                say("    to the curve phase's middle (%.0f, %.0f) for room" % (c[0], c[1]))
                cam = self.goto((c[0], c[1]), "R", 0, cam=cam, odometry=False, name="the curve phase's middle")
                hp = self.arc_plan_heading(cam, k, L, v)
        if hp is None:
            row.update(result="skipped", why="no start heading gives it room here or in the middle (%s)" %
                       self.arc_room(cam[0], cam[1], cam[2], k, L, v)[1])
            return self.arc_log(row)
        cam = self.arc_turn_to(cam, hp[0])
        m, w = self.arc_room(cam[0], cam[1], cam[2], k, L, v)
        if m < 0:                                   # the turn ended off the planned heading: once more
            hp = self.arc_plan_heading(cam, k, L, v)
            if hp is None:
                row.update(result="skipped", why="after the turn: %s" % w)
                return self.arc_log(row)
            cam = self.arc_turn_to(cam, hp[0])
            if self.arc_room(cam[0], cam[1], cam[2], k, L, v)[0] < 0:
                row.update(result="skipped", why="after the turn: %s" % w)
                return self.arc_log(row)
        row.update(cam0=[round(q, 2) for q in cam], margin_cm=round(m, 1))
        d = self.drive_arc(cam, k, L, v)
        row.update(result=d["result"], why=d["why"], own0=d["own0"], own1=d["own1"])
        time.sleep(SETTLE_S)
        cam1 = self.cam_pose()
        if cam1 is None:
            # the camera lost the tag on the arc: the robot's own pose (in the camera's frame since the fpose) is the
            # estimate; the calibration's recovery drives it back towards where the tag read well, bubble-checked
            own = d.get("own_last") or d.get("own1")
            last = self.eye.last(self.tag)
            if own is not None and self.a.lost_policy == "recover":
                blind = math.hypot(own[0] - last[1], own[1] - last[2]) * 1.2 if last else L
                self.recover((own[0], own[1], own[2]), blind, math.degrees(abs(k) * blind), "a curve (radius %g)" % R)
            else:
                self.hold_until_seen()
            cam1 = self.cam_pose()
            if cam1 is None:
                self.hold_until_seen()
                cam1 = self.cam_pose()
            row["why"] = (row.get("why") or "") + "; the tag was lost on it"
        self.reanchor(cam1, "after a curve")
        row["cam1"] = [round(q, 2) for q in cam1] if cam1 else None
        if d["own0"] and d["own1"] and self.Draw is not None:
            self.Draw = compose(self.Draw, relative(d["own0"], d["own1"]))
        if d["result"] == "done" and d["t0"] is not None:
            ox, oy = self.offset
            pts = []
            for (_t, x, y, th) in self.eye.frame_window(self.tag, d["t0"], d["t1"]):
                c, s_ = math.cos(math.radians(th)), math.sin(math.radians(th))
                pts.append((x - (c * ox - s_ * oy), y - (s_ * ox + c * oy)))
            row["n"] = len(pts)
            f = FC.fit_arc_points(pts, k, L, min_k=ARC_FIT_MIN_K)
            if f is not None:
                row.update(k_ratio=round(f[0], 4), length_ratio=round(f[1], 4), R_fit=round(f[2], 2))
                if pts:
                    row["edge_cm_min"] = round(min(self.S.edge_dist(p_) for p_ in pts), 1)
            else:
                row["why"] = "not fitted (too few camera reads: %d)" % len(pts)
        return self.arc_log(row)

    def arc_log(self, row):
        self.arc_rows.append(row)
        new = not os.path.exists(os.path.join(self.folder, "arcs.csv"))
        cols = ["round", "use", "radius_cm", "side", "k", "L_cm", "v_cms", "result", "why", "n", "k_ratio",
                "length_ratio", "R_fit", "margin_cm", "edge_cm_min", "cam0", "cam1", "own0", "own1"]
        with open(os.path.join(self.folder, "arcs.csv"), "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(cols)
            w.writerow([json.dumps(row.get(c)) if isinstance(row.get(c), (list, tuple)) else row.get(c, "")
                        for c in cols])
        say("    arc %s r %g %s at %.1f cm/s: %s%s" % (row["use"], row["radius_cm"], row["side"], row["v_cms"],
                                                     row["result"] or "?", (": curvature x%.3f, length x%.3f" % (
                                                         row["k_ratio"], row["length_ratio"])) if row.get("k_ratio")
                                                     else (" (%s)" % row["why"] if row.get("why") else "")))
        self.ev("arc", **row)
        return row

    def phase_r(self):
        """(11-10b) The curve phase: ARC_ROUNDS x (3 radii x left / right x 2 speeds) arcs, fitted, the arc scales
        saved when the fits agree (free roam's rule), then ARC_VERIFY arcs with the new values. Writes arcs.csv and
        arcs.json. Never ends the run by itself: a phase that cannot fit says why and the tours go on."""
        import freeroam_core as FC
        self.arc_rows = []
        on, why = self.arc_wanted()
        self.arc_result = {"on": on, "why": why}
        self.ev("arc_phase", on=on, why=why)
        if not on:
            say("CURVE PHASE for %s: not driven: %s" % (self.name, why))
            self.set_status(arc_phase="not driven: " + why)
            return self.arc_result
        say("CURVE PHASE for %s: %s" % (self.name, why))
        self.set_status(phase="R", words="Curve phase: arcs left and right, radius 12, 25 and 50 cm, at 5 and 10 cm/s",
                        arc_phase="driving")
        cur = (self.rb.get("arc_k_scale"), self.rb.get("arc_d_scale"))
        if None in cur:
            self.arc_result.update(on=False, why="it did not answer get arc_k_scale / arc_d_scale")
            say("  CURVE PHASE skipped: " + self.arc_result["why"])
            return self.arc_result
        coast = self.rb.get("arc_coast_ms")
        self.rb.set_checked("arc_coast_ms", 0.0)    # brake at each arc's end (no coast round the same curve)
        fits = []
        try:
            c = self.arc_centre()
            self.arc_result["centre"] = [round(c[0], 1), round(c[1], 1), round(c[2], 1)] if c else None
            if c is None:
                raise RunEnd("no spot on the table has room for the curves and reads well")
            say("  the curve phase's middle (where the camera reads best, furthest from edges, outlets and blind "
                "spots): %.0f, %.0f (%.0f cm of room)" % c)
            self.goto((c[0], c[1]), "R", 0, odometry=False, name="the curve phase's middle")
            n = 0
            for rnd in range(1, ARC_ROUNDS + 1):
                for v in ARC_SPEEDS:
                    for R in ARC_RADII_CM:
                        for sg in (1.0, -1.0):
                            if self.stop_flag.is_set():
                                raise RunEnd("stopped")
                            n += 1
                            if n % BATTERY_CHECK_EVERY == 0:
                                self.check_battery()
                            self.set_status(words="Curve phase: arc %d of %d (radius %g %s, %.1f cm/s)" % (
                                n, ARC_ROUNDS * 12, R, "left" if sg > 0 else "right", min(v, FC.v_cap(1 / R))))
                            row = self.one_arc(rnd, R, sg, min(v, FC.v_cap(1 / R)), "fit")
                            if row.get("k_ratio"):
                                fits.append((row["k_ratio"], row["length_ratio"]))
            res = FC.arc_summary(fits)
            out = {"before": {"arc_k_scale": cur[0], "arc_d_scale": cur[1]}, "fits": len(fits),
                   "arcs": sum(1 for r_ in self.arc_rows if r_["use"] == "fit")}
            if res is None:
                out.update(saved=False, words="only %d arcs fitted: nothing to decide" % len(fits))
            else:
                kr, kh, dr, dh, nn = res
                ok, ks, ds = FC.arc_agree(res, cur)
                out.update(k_ratio=round(kr, 4), k_ci=round(kh, 4), length_ratio=round(dr, 4), length_ci=round(dh, 4),
                           new={"arc_k_scale": round(ks, 4), "arc_d_scale": round(ds, 4)}, agree=ok)
                w = ("%d arcs fitted: it turns x%.3f (+-%.3f) the curvature asked and runs x%.3f (+-%.3f) the length; "
                     "arc_k_scale %.4f -> %.4f, arc_d_scale %.4f -> %.4f" % (nn, kr, kh, dr, dh, cur[0], ks, cur[1], ds))
                set_ok = False
                if ok and self.a.save_on_robot:
                    o1, g1 = self.rb.set_checked("arc_k_scale", round(ks, 4))
                    o2, g2 = self.rb.set_checked("arc_d_scale", round(ds, 4))
                    sv = None
                    if o1 and o2:
                        k_ = self.link.mark()
                        self.link.send(self.rb.ip, "save")
                        sv = self.link.wait_for(lambda n_, l: l if n_ == self.rb.name and (
                            "saved on the robot" in l or "unknown command" in l or "refused" in l) else None, k_, 4.0)
                    set_ok = bool(o1 and o2)
                    saved = bool(sv and "saved on the robot" in sv)
                    out.update(saved=saved, readback={"arc_k_scale": g1, "arc_d_scale": g2})
                    w += ": SET%s" % (" and SAVED on the robot (save)" if saved else
                                      " but NOT saved (%s)" % (sv or "no answer to save" if o1 and o2 else
                                                               "they did not read back"))
                elif ok:
                    out.update(saved=False)
                    w += ": NOT set (saving on the robot is switched off: --no-save)"
                else:
                    out.update(saved=False)
                    w += ": NOT set (free roam's rule: %d+ arcs, both within +-%.2f, new values 0.8-1.25)" % (
                        FC.ARC_FIT_MIN, FC.ARC_FIT_CI)
                out["words"] = w
                if set_ok:
                    chk = []
                    for R in ARC_RADII_CM[:ARC_VERIFY]:
                        sg = 1.0 if len(chk) % 2 == 0 else -1.0
                        row = self.one_arc(0, R, sg, min(ARC_SPEEDS[-1], FC.v_cap(1 / R)), "check")
                        if row.get("k_ratio"):
                            chk.append((row["k_ratio"], row["length_ratio"]))
                    if chk:
                        out["check"] = {"n": len(chk), "k_ratio": round(float(np.median([c[0] for c in chk])), 4),
                                        "length_ratio": round(float(np.median([c[1] for c in chk])), 4)}
                        out["words"] += "; with the new values %d check arcs ran x%.3f curvature, x%.3f length" % (
                            len(chk), out["check"]["k_ratio"], out["check"]["length_ratio"])
            self.arc_result.update(out)
            say("  CURVES: " + out.get("words", ""))
        except RunEnd as e:
            if self.stop_flag.is_set():
                raise
            self.arc_result.update(words="the curve phase stopped: %s (the tours go on)" % e)
            say("  CURVE PHASE STOPPED: %s; the tours go on" % e)
        finally:
            if coast is not None:
                self.rb.set_checked("arc_coast_ms", coast)
            with open(os.path.join(self.folder, "arcs.json"), "w") as f:
                json.dump(dict(self.arc_result, rows=self.arc_rows), f, indent=1, default=str)
        self.set_status(arc_phase=self.arc_result.get("words") or self.arc_result.get("why"))
        return self.arc_result

    # ------------------------------------------------------------------------------------------------- the whole run
    def start_robot(self):
        self.set_status(phase="find", words="Finding the robot on the WiFi (waking it if it sleeps)")
        found = discover(self.link, self.a.robot, self.a.wake_min, self.a.dry, count=self.a.count,
                         only=tuple(x.strip() for x in (self.a.only or "").split(",") if x.strip()))
        if self.a.count and len(found) < self.a.count:
            say("NOTE: %d of the %d robots expected answered (%s): calibrating those" % (
                len(found), self.a.count, ", ".join(found) or "none"))
        if not found:
            raise RunEnd("no robot answered on the WiFi (sent status and wake to %s for %.0f min). Is it switched on "
                         "and on AGV_SWARM?" % (", ".join(self.a.robot) or "the broadcast", self.a.wake_min))
        say("Found: %s" % ", ".join("%s at %s" % kv for kv in found.items()))
        # (Nick, 5 Nov 11:00) a robot on older firmware is refused, said plainly (switch calibration.min_firmware); it is
        # not moved, and its roof tag is kept out like a parked robot's
        self.fw, self.refused = {}, {}
        minv = getattr(self.a, "min_firmware", "")
        for n, ip in list(found.items()):
            fw = firmware_of(Robot(self.link, n, ip))
            self.fw[n] = fw
            if not firmware_ok(fw, minv):
                w = firmware_refusal(n, fw, minv)
                self.refused[n] = w
                del found[n]
                say("REFUSED: %s. It is not moved; its roof tag is kept out where it stands." % w)
                self.ev("refused_firmware", robot=n, firmware=fw, need=minv)
            else:
                say("  %s: firmware %s%s" % (n, fw or "(not said)", " (needs %s or newer: OK)" % minv if minv else ""))
        if self.refused:
            self.set_status(refused=list(self.refused.values()))
        if not found:
            raise RunEnd("every robot that answered is on older firmware: %s. Nothing moved." % "; ".join(
                self.refused.values()))
        self.robots = [Robot(self.link, n, ip) for n, ip in found.items()]
        self.wifi = {rb.name: wifi_name_of(self.link, rb) for rb in self.robots}
        for rb in self.robots:
            rb.link.send(rb.ip, "wake")
            guard = [("set safe_edge %d" % self.a.safe_edge) if c.startswith("set safe_edge") else c for c in EDGE_GUARD]
            for c in guard + ["set turn_max_rpm %.1f" % TURN_RPM]:
                rb.link.send(rb.ip, c)
                time.sleep(0.05)
        for rb in self.robots:
            self.ensure_compass(rb)
        # (11-05h, Nick 20:46) the robot-side safety layers switched on and READ BACK (tilt stop, bottom-zone drop):
        # a robot whose layers do not read back is not moved
        for rb in list(self.robots):
            ok, words = arm_safety(rb, self.a.safe_edge)
            self.ev("safety_armed", robot=rb.name, ok=ok, words=words)
            if not ok:
                self.refused[rb.name] = "%s: its safety layers did not read back (%s)" % (rb.name, words)
                say("REFUSED: %s. It is not moved." % self.refused[rb.name])
                self.robots.remove(rb)
                found.pop(rb.name, None)
        if not self.robots:
            raise RunEnd("no robot's safety layers read back: nothing moved")
        return found

    def ensure_compass(self, rb):
        self.set_status(words="Checking %s can turn (compass check since it woke)" % rb.name)
        ok, how = compass_check(self.link, rb)
        self.ev("compass_check", robot=rb.name, ok=ok, how=how)
        return ok

    # ------------------------------------------------------------------------------------------------- (e14) finding
    def motion_cams(self):
        """(e14) Every camera for robot finding by motion: the simulator's pictures (drawn from its true poses) in a
        dry run; on the Pi every camera found (the recorder's streams, then /dev/video cameras it does not hold), the
        first with the setup's camera model. Returns ([CamInfo], [closers])."""
        import camera_discovery as CD
        if self.a.dry:
            import motion_find as MF
            cam = getattr(self, "sim_cam", None)
            spec = [dict(name="overhead", pos=tuple(cam.C) if cam is not None else (-114.3, self.S.W / 2.0, 145.0),
                         look=tuple(getattr(self.a, "sim_look", None) or (150.0, self.S.W / 2.0, 0.0)), hfov=70.4)]
            sp = MF.SimPictures(self.eye.truth_now, getattr(self, "sim_tags", {}) or {}, (self.S.L, self.S.W),
                                cams=spec, darks=getattr(self.eye, "darks", []) or [], unread=self.eye.unread_now,
                                extra_blind=bool(getattr(self.a, "sim_extra_camera", False)))
            return sp.caminfos(), []
        from cameras import CameraModel
        cal = getattr(self.a, "cal_used", None)
        model = CameraModel.load(cal) if cal and os.path.exists(cal) else None
        found = CD.find_cameras()
        grabs = [CD.Grabber(c["src"]) for c in found]
        cams = [CD.CamInfo(c["name"], g.grab, model if (k == 0 and c["kind"] == "stream") else None)
                for k, (c, g) in enumerate(zip(found, grabs))]
        return cams, [g.close for g in grabs]

    def clear_space_ok(self, words):
        """(e14) Did the person confirm the clear space for the strokes? --clear-space yes / no; auto: a dry run yes,
        MIRLT_CLEAR_SPACE_OK=1 (Mission Control's Calibrate card ticks it) yes, a terminal asks, else no (every robot
        then wiggles on the spot instead: only its own footprint)."""
        v = str(getattr(self.a, "clear_space", "auto") or "auto")
        if v in ("yes", "no"):
            return v == "yes"
        if self.a.dry and hasattr(getattr(self, "eye", None), "truth_now"):
            # the simulator's person looks at the table: robots with that much room from every edge (the truth)
            R = float(getattr(self, "_clear_r", 19.0))
            return {n for n, p_ in self.eye.truth_now().items() if min(p_[0], p_[1], self.S.L - p_[0],
                                                                       self.S.W - p_[1]) >= R}
        if self.a.dry or os.environ.get("MIRLT_CLEAR_SPACE_OK") == "1":
            return True
        if sys.stdin is not None and sys.stdin.isatty():
            try:
                return input("Has every robot that much clear space round it? (y/n) ").strip().lower().startswith("y")
            except EOFError:
                return False
        return False

    def find_by_motion(self, found):
        """(e14, Nick 10 Nov) Each robot found by its own small move, anywhere on the table, with or without a roof tag
        that reads (motion_find.py); a robot found where its tag does not read drives itself back (never give up).
        Returns {wifi name: tag id} for the robots whose tag moved with their own stroke (bound by it)."""
        import motion_find as MF
        import camera_discovery as CD
        if not MF.enabled() or self.a.binding == "fixed":
            return {}
        self.set_status(phase="find", words="Finding each robot by its own small move (the motion detector)")
        try:
            cams, closers = self.motion_cams()
        except Exception as e:  # noqa: BLE001  (finding by motion never stops a run: the wiggle binding follows)
            say("(robot finding by motion skipped: no cameras opened: %s)" % e)
            return {}
        try:
            if not cams:
                say("(robot finding by motion skipped: no camera found)")
                return {}
            cpp = None
            m0 = next((c.model for c in cams if c.model is not None), None)
            if m0 is not None:
                try:
                    cpp = 1.0 / float(m0.px_per_cm(self.S.L / 2.0, self.S.W / 2.0, 4.0))
                except Exception:  # noqa: BLE001
                    cpp = None
            cl = CD.clearance(cpp)
            say("CLEAR SPACE (finding each robot by its own small move): " + cl["words"])
            self._clear_r = cl["radius_cm"]
            ok = self.clear_space_ok(cl["words"])
            if isinstance(ok, set):
                rest_ = sorted(set(rb.name for rb in self.robots) - ok)
                say("  clear space confirmed for %s; %s %s on the spot instead (only its own footprint)" % (
                    ", ".join(sorted(ok)) or "no robot", ", ".join(rest_) or "none",
                    "wiggles" if len(rest_) == 1 else "wiggle"))
            elif not ok:
                say("  the clear space was not confirmed: each robot WIGGLES on the spot instead of a stroke (+-%.0f "
                    "deg: only its own footprint)" % CD.WIGGLE_DEG)
            robots = {rb.name: rb for rb in self.robots}
            for rb in self.robots:
                rb.set("e02_cms", CD.DISC_SPEED)
            # a pose guess from a pinned roof tag read where it stands: a stroke only where that guess shows it clear
            pins = load_pins(self.site, state_path(self.a, PINS_FILE)) if getattr(self.a, "pins", True) else {}
            wf = getattr(self, "wifi", {}) or {}
            guesses = {}
            for rb in self.robots:
                t = (pins.get(wf.get(rb.name)) or {}).get("tag")
                l_ = self.eye.last(int(t)) if t is not None else None
                if l_ is not None and time.time() - l_[0] < 3.0:
                    guesses[rb.name] = (l_[1], l_[2], l_[3])
            def rm_tag(t):
                old = getattr(self, "tag", None)
                self.tag = t
                try:
                    return self.readmap_tag()
                finally:
                    if old is None:
                        del self.tag
                    else:
                        self.tag = old
            pre, res = MF.run_finding(robots, self.eye, cams, self.S, self.readmap(),
                                      ok if isinstance(ok, set) else (True if ok else set()), guesses, log=say,
                                      ev=self.ev, stop=self.stop_flag.is_set, hold_say_s=HOLD_SAY_S,
                                      turn_dps=TURN_DPS, final_cms=FINAL_CMS, folder=self.folder, rm_tag=rm_tag)
            res["clearance"] = cl
            self.motion_found = res
            if res.get("tagless"):
                say("  NOT CALIBRATED: %s (no roof tag reads on %s: the calibration measures by the tag)" % (
                    ", ".join(res["tagless"]), "it" if len(res["tagless"]) == 1 else "them"))
                self.robots = [rb for rb in self.robots if rb.name not in res["tagless"]]
                if not self.robots:
                    raise RunEnd("no robot here has a roof tag that reads (found by motion: %s); the calibration "
                                 "measures by the tag" % ", ".join(res["tagless"]))
            return pre
        finally:
            for c in closers:
                try:
                    c()
                except Exception:  # noqa: BLE001
                    pass
            self.S.set_parked({})

    def bind(self, found, pre=None):
        self.set_status(phase="bind", words="Binding: each robot does the sync wiggle; the tag that follows it is its")
        if self.a.binding == "fixed":
            names = [site_name_for(self.site, n, ip) for n, ip in found.items()]
            b = fixed_binding(self.site, names)
            how = "fixed (the site's tags.ids)"
        else:
            import experiments as EX
            w = EX.conf(self.site)["sync_wiggle"]
            # (5 Nov 11:00) pinned robots: the check wiggle; their pinned tag must turn (else re-bound, said loudly)
            pp = state_path(self.a, PINS_FILE)
            pins = load_pins(self.site, pp) if getattr(self.a, "pins", True) else {}
            wf = getattr(self, "wifi", {}) or {}
            expect = {rb.name: pins[wf.get(rb.name)]["tag"] for rb in self.robots
                      if wf.get(rb.name) in pins and pins[wf.get(rb.name)].get("tag") is not None}
            if expect:
                say("Pinned (robot_pins.json): %s: a %.0f deg check wiggle each" % (", ".join(
                    "%s = tag %s" % kv for kv in expect.items()), PIN_CHECK_DEG))
            self.pin_notes = {}
            pre = dict(pre or {})
            for n_, t_ in pre.items():
                # (e14) bound by its own stroke: the roof tag whose reads moved with it (motion_find)
                bb_ = ((getattr(self, "motion_found", {}) or {}).get("robots", {}).get(n_) or {}).get("bound_by")
                self.pin_notes[n_] = "found by motion: " + (bb_ or "tag %d moved with its own stroke" % t_)
                say("  %s: %s" % (n_, self.pin_notes[n_]))
            rest = [rb for rb in self.robots if rb.name not in pre]
            b0 = dict(pre)
            if rest:
                b0.update(bind_by_wiggle(rest, self.eye, wiggle_deg=float(w["deg"]), pause_s=float(w["pause_s"]),
                                         expect={k: v for k, v in expect.items() if k not in pre},
                                         check_deg=PIN_CHECK_DEG, notes=self.pin_notes))
            if getattr(self.a, "pins", True):
                update_pins(pp, pins, {wf.get(n): t for n, t in b0.items()})
            b = {site_name_for(self.site, n, found[n]): t for n, t in b0.items()}
            how = "wiggle (experiments.py sync wiggle, %.0f deg and back; pinned robots a %.0f deg check)" % (
                float(w["deg"]), PIN_CHECK_DEG)
            if pre:
                how = ("motion (each robot's own finding stroke moved its tag)" if not rest else
                       "motion for %s; %s" % (", ".join(sorted(pre)), how))
        notes = getattr(self, "pin_notes", {}) or {}
        wf = getattr(self, "wifi", {}) or {}
        rec = {"how": how, "robots": [{"name": site_name_for(self.site, rb.name, rb.ip), "wifi_name": rb.name,
                                       "wifi": wf.get(rb.name), "firmware": (getattr(self, "fw", {}) or {}).get(rb.name),
                                       "pin": notes.get(rb.name),
                                       "ip": rb.ip, "tag_id": b.get(site_name_for(self.site, rb.name, rb.ip))}
                                      for rb in self.robots], "time": dt.datetime.now().isoformat(timespec="seconds"),
               "version": VERSION, "refused": dict(getattr(self, "refused", {}) or {})}
        with open(os.path.join(self.folder, "binding.json"), "w") as f:
            json.dump(rec, f, indent=1)
        if not self.a.dry:
            cd = self.site.get("conf_dir")
            if cd:
                os.makedirs(cd, exist_ok=True)
                with open(os.path.join(cd, "robot_binding.json"), "w") as f:
                    json.dump(rec, f, indent=1)
        say("Bound (%s): %s" % (how, ", ".join("%s = tag %s" % (r["name"], r["tag_id"]) for r in rec["robots"])))
        self.ev("bound", **rec)
        return rec

    def run(self, eye_factory):
        self.rows = []
        self.link = Link(self.folder)
        found = self.start_robot()
        self.eye = eye_factory()
        time.sleep(2.0)
        pre = self.find_by_motion(found)      # (e14) each robot found by its own small move; brought back if needed
        rec = self.bind(found, pre)
        self.apply_pending_evidence()
        self.binding = rec
        # (5 Nov) a roof tag the camera sees that no robot claimed (a robot that did not answer, switched off or
        # asleep, refused for its firmware, or a spare lying there): kept out like a parked robot, where it is now
        bound = {int(r["tag_id"]) for r in rec["robots"] if r.get("tag_id") is not None}
        self.unclaimed = {}
        for t in self.eye.seen_ids(2.0):
            if t not in bound:
                m = self.eye.measure(t, 0.6)
                if m:
                    self.unclaimed["(tag %d, no robot in this run for it)" % t] = (m[0], m[1])
        if self.unclaimed:
            say("Kept out: %s" % ", ".join("%s at %.0f, %.0f" % (n, p_[0], p_[1]) for n, p_ in self.unclaimed.items()))
            self.ev("unclaimed_tags", tags=self.unclaimed)
        plan = self.order_robots(rec)
        if len(plan) == 1:
            rb, name, tag, spot = plan[0]
            self.S.park_label = spot
            self.S.set_parked(self.unclaimed)
            self.rb, self.name, self.tag, self.k, self.n = rb, name, tag, 0, 1
            self.set_status(robots=[{"name": name, "wifi_name": rb.name, "tag": tag, "spot": spot}])
            return self.calibrate_one()
        # (Nick, 5 Nov) every robot found, one after another: each with its own folder, fit, save and report section;
        # the others parked at their own spots and kept out of its way; each parks and sleeps when it is done
        say("%d robots: calibrated one after another (%s)" % (len(plan), ", ".join(
            "%s at %s" % (p_[1], p_[3]) for p_ in plan)))
        self.set_status(robots=[{"name": n, "wifi_name": rb.name, "tag": t, "spot": sp} for rb, n, t, sp in plan])
        subs = []
        for k, (rb, name, tag, spot) in enumerate(plan):
            sub = CalibRun(self.a, self.site, os.path.join(self.folder, re.sub(r"[^A-Za-z0-9_-]", "_", name)),
                           park_label=spot, parent=self)
            sub.link, sub.eye, sub.robots, sub.stop_flag, sub.binding = self.link, self.eye, self.robots, \
                self.stop_flag, rec
            sub.rb, sub.name, sub.tag, sub.k, sub.n, sub.rows = rb, name, tag, k, len(plan), []
            sub.wifi = getattr(self, "wifi", {})
            for at in ("cam_centre", "tag_h", "sim_true", "sim_cam"):
                if hasattr(self, at):
                    setattr(sub, at, getattr(self, at))
            if getattr(self, "S_quick", False):
                sub.S.zone12 = self.S.zone12            # --quick: the short tour for every robot
            if getattr(self, "sim_true_by", None):
                sub.sim_true = self.sim_true_by.get(rb.name, getattr(self, "sim_true", None))
            subs.append(sub)
        self.subs = subs
        results = []
        import calib_report as CRP
        for k, sub in enumerate(subs):
            if self.stop_flag.is_set():
                results.append("not started (stopped)")
                continue
            try:
                for o in subs[k + 1:]:
                    # the robots still to come go to their own spots first, out of its way (by the camera)
                    o.S.set_parked(self.where([q for q in subs if q is not o]))
                    o.stage()
                sub.S.set_parked(self.where([q for q in subs if q is not sub]))
                say("=== %s (robot %d of %d): calibrated at %s; kept out round %s ===" % (
                    sub.name, k + 1, len(subs), sub.S.park_label, ", ".join(
                        "%s" % kk["name"] for kk in sub.S.keepouts if kk.get("kind") == "robot") or "nobody"))
                res = sub.calibrate_one()
            except (RunEnd, LowBattery) as e:
                res = "ended: %s" % e
                say("ENDED (%s): %s" % (sub.name, e))
            sub.result = getattr(sub, "result", res)
            results.append(res)
            for f in (sub.legs_f, sub.raw_f, sub.cor_f, sub.events):
                try:
                    f.flush()
                except (OSError, ValueError):
                    pass
            try:
                sub.set_status(phase="done", result=res, ended=time.time(), closest=sub.closest)
                CRP.write(sub.folder, sub, self.a)
            except Exception as e:  # noqa: BLE001
                say("(the report for %s could not be written: %s)" % (sub.name, e))
        self.result = "finished" if all(r == "finished" for r in results) else "; ".join(
            "%s: %s" % (s_.name, r) for s_, r in zip(subs, results))
        self.closest = {kk: (min if kk != "view_x_max" else max)(s_.closest[kk] for s_ in subs)
                        for kk in subs[0].closest}
        self.parked = subs[-1].parked if hasattr(subs[-1], "parked") else None
        self.slept = all(getattr(s_, "slept", False) for s_ in subs)
        self.ev("end", result=self.result, robots=[s_.name for s_ in subs])
        return self.result

    def order_robots(self, rec):
        """[(Robot, name, tag, spot)]: the robot nearest PARK_SPOTS[0] first (it is calibrated there), the others to
        the next spots in turn, nearest first. More robots than spots: the extra ones are left out, said."""
        byw = {r["wifi_name"]: r for r in rec["robots"]}
        rows = []
        for rb in self.robots:
            r = byw.get(rb.name) or {}
            if r.get("tag_id") is None:
                continue
            m = self.eye.measure(int(r["tag_id"]), 0.6)
            rows.append([rb, r["name"], int(r["tag_id"]), (m[0], m[1]) if m else None])
        spots = [p for p in (getattr(self.a, "park_spots", None) or PARK_SPOTS) if p in self.S.points]
        # (5 Nov 11:00) a spot where a robot nobody drives stands (refused firmware, asleep, a spare tag) is not used
        for nm_, (ux, uy) in (getattr(self, "unclaimed", {}) or {}).items():
            for sp in list(spots):
                if math.hypot(self.S.points[sp][0] - ux, self.S.points[sp][1] - uy) < PARKED_R_CM + KEEPOUT_CLEAR_CM:
                    spots.remove(sp)
                    say("NOTE: spot %s is not used: %s stands there" % (sp, nm_))
        out = []
        left = list(rows)
        for sp in spots:
            if not left:
                break
            sx, sy = self.S.points[sp]
            left.sort(key=lambda q: math.hypot(q[3][0] - sx, q[3][1] - sy) if q[3] else 1e9)
            q = left.pop(0)
            out.append((q[0], q[1], q[2], sp))
        for q in left:
            say("NOTE: %s is not calibrated in this run (only %d park spots: %s); it must stay still where it is" % (
                q[1], len(spots), ", ".join(spots)))
        return out

    def where(self, subs):
        """{name: (x, y)} of these robots where the camera sees them now (and the unclaimed tags)."""
        out = dict(getattr(self, "unclaimed", {}))
        for o in subs:
            m = self.eye.measure(o.tag, 0.6)
            if m:
                out[o.name] = (m[0], m[1])
            else:
                say("  (%s is not seen by the camera now: not kept out)" % o.name)
        return out

    def stage(self):
        """(5 Nov) Drive to its own spot by the camera (safety) if it is more than AT_SPOT_CM from it, facing the park
        heading; the other robots are kept out."""
        cam = self.cam_pose_sure()                 # (e13, never give up) waits, asking for a hand, not an end
        x, y = self.S.points[self.S.park_label]
        if self.D is None:
            self.reanchor(cam, "start")
        if math.hypot(cam[0] - x, cam[1] - y) <= AT_SPOT_CM:
            return cam
        say("%s goes to its spot %s first (%.0f cm away), out of the way" % (
            self.name, self.S.park_label, math.hypot(cam[0] - x, cam[1] - y)))
        self.set_status(words="Going to its spot %s first" % self.S.park_label)
        cam, miss = self.park("stage")
        return cam

    def heading_check(self, cam):
        """(11-05f, school 5 Nov 18:54; 11-05h: the shared heading_nudge) Which way the robot really faces, against its
        roof tag's heading: a short nudge, and the direction the camera saw it move. The tag on AL-1DC0 reads 180 deg
        off its nose, so every leg was aimed backwards and the robot ended 1 cm from the TV-end edge. The nudge is
        driven only if it is clear both ways, so a wrong heading cannot take it over an edge.
        (e15, school 10 Nov 18:54) A lost tag no longer ends it: heading_check_patient tries again where the tag reads,
        then the remembered turn (tag_heading_store); a measured turn is remembered. Nothing measured and nothing
        remembered: measured again (never give up). MIRLT_HEADING_CHECK=once: the check once, as before."""
        import tag_heading_store as THS
        hp = state_path(self.a, THS.FILE) if THS.enabled(self.site) else ""
        if os.environ.get("MIRLT_HEADING_CHECK", "patient") != "once":
            while True:
                res = heading_check_patient(self.rb, self.eye, self.tag, cam,
                                            lambda a, b: self.S.problems(a, b, buf=2.0, edge_buf=2.0),
                                            stop=self.stop_flag.is_set, measure=self.cam_pose,
                                            remembered=THS.get(hp, self.tag) if hp else None)
                if res["state"] != "unknown":
                    break
                if res.get("stage") == "clear" or self.stop_flag.is_set():
                    raise RunEnd("the heading check needs %.0f cm clear both ways from where %s stands (ahead and "
                                 "behind, or to both sides): move it nearer the middle of the table and start again"
                                 % (6.0, self.name) if res.get("stage") == "clear" else "stop asked")
                say("  %s: its heading is not known yet: measured again (never give up)" % self.name)
                cam = self.cam_pose_sure()
            use = float(res["offset"] or 0.0)
            self.ev("heading_check", moved_cm=res.get("moved"), seen_deg=res.get("raw"), offset_deg=use, tag=self.tag,
                    state=res["state"], how=res["how"])
            self.status["tag_heading_offset_deg"] = use
            self.status["tag_heading_state"] = res["state"]
            if res["state"] == "measured" and hp:
                say("  " + THS.record(hp, self.tag, use, "the calibration's heading check", robot=self.rb.name)["words"])
            import observe as OB
            self.hwatch = OB.HeadingWatch(0.0)
            time.sleep(1.5 if abs(use) >= 1.0 else 0.0)
            return self.cam_pose_sure()
        use, raw, mv, c1 = heading_nudge(self.rb, self.eye, self.tag, cam,
                                         lambda a, b: self.S.problems(a, b, buf=2.0, edge_buf=2.0),
                                         stop=self.stop_flag.is_set, measure=self.cam_pose)
        self.ev("heading_check", moved_cm=round(mv, 2), seen_deg=round(raw, 1), offset_deg=use, tag=self.tag)
        self.status["tag_heading_offset_deg"] = use
        import observe as OB
        self.hwatch = OB.HeadingWatch(0.0)       # (11-05h) checked again on every straight leg from here on
        if abs(use) >= 1.0:
            time.sleep(1.5)
            c1 = self.cam_pose_sure()              # (e13, never give up)
        return c1

    def readmap(self):
        """(11-05h) This run's readability map (shared by the robots of one run through the parent): every leg's end
        read or not by the camera, where the robot was."""
        host = getattr(self, "parent", None) or self
        if getattr(host, "rm", None) is None:
            import observe as OB
            host.rm = OB.ReadMap(self.S.L, self.S.W)
            host.rm.seed(lambda x_, y_: 0.9, "taken as readable until the run learns otherwise")
            if READMAP_SEED:
                # (11-10b, e12 item 2; Observe then trust) the camera model's cautious guess as the first guess, then
                # what earlier runs here learned (the site's readmap.json, half weight): blind areas learned per site
                try:
                    host.rm_words = seed_readmap(host.rm, self.site, getattr(self, "sim_cam", None), getattr(
                        self.a, "cal_used", None), dry=bool(self.a.dry), state_dir=getattr(self.a, "state_dir", ""),
                        grey=self.light_now())
                    say("READABILITY: %s" % host.rm_words)
                except Exception as e:  # noqa: BLE001  (never stops a run: the old seed stays)
                    say("(readability seed: %s; every spot taken as readable until the run learns otherwise)" % e)
            import readmap_store as RS
            host.rm_base = RS.snapshot(host.rm)       # (e14) this run's own evidence is what comes after
        return host.rm

    def light_now(self):
        """(e14) The light now: --sim-light (a dry run), else the bridge's brightness.csv so far (median)."""
        if getattr(self.a, "sim_light", None) is not None:
            return float(self.a.sim_light)
        import readmap_store as RS
        host = getattr(self, "parent", None) or self
        return RS.grey_of(getattr(host, "folder", "") or "")

    def apply_pending_evidence(self):
        """(e14) The spots where a robot was found by motion and its tag did not read: added to that tag's own map once
        the binding knows the tag (3 tries, none read, as a skipped point)."""
        host = getattr(self, "parent", None) or self
        for t, spots in getattr(self, "pending_tag_evidence", []) or []:
            if t is None:
                continue
            old = getattr(self, "tag", None)
            self.tag = t
            try:
                m = self.readmap_tag()
                for (fx, fy) in spots[:1]:
                    for _ in range(3):
                        m.add(fx, fy, False)
            finally:
                if old is None:
                    del self.tag
                else:
                    self.tag = old
        host.pending_tag_evidence = []

    def readmap_tag(self):
        """(e13) This robot's roof tag's own readability evidence (a point may read for one tag and not another): no
        seed of its own (a weak 50/50), earlier runs' reads of this tag at this site carried in (readmap_tag<id>.json,
        half weight). Used where it has 3+ tries at a point; saved at the end beside the site's map."""
        host = getattr(self, "parent", None) or self
        maps = host.__dict__.setdefault("rm_tags", {})
        if self.tag not in maps:
            import observe as OB
            m = OB.ReadMap(self.S.L, self.S.W)
            folder = getattr(self.a, "state_dir", "") if self.a.dry else ((self.site or {}).get("conf_dir") or "")
            p = os.path.join(folder, "readmap_tag%s.json" % self.tag) if folder else ""
            import readmap_store as RS
            if RS.enabled() and folder:
                # (e14) this tag's own sessions, chosen for the light now (readmap_store)
                w_ = RS.choose(folder, m, self.light_now(), self.tag, only_tag=True)
                say("READABILITY for tag %s: %s" % (self.tag, w_))
                p = ""
                host.__dict__.setdefault("rm_tag_base", {})[self.tag] = RS.snapshot(m)
            if p and os.path.exists(p):
                try:
                    with open(p) as f:
                        m.merge_history(OB.ReadMap.from_json(json.load(f)))
                except (OSError, ValueError, KeyError):
                    pass
            maps[self.tag] = m
        return maps[self.tag]

    def after_trip(self, r, cam0):
        """(11-05h, Nick 20:46) A leg stopped by the robot's tilt stop or its edge guard's drop: that spot is a
        discovered edge (a keep-out for the rest of the run), the robot backs straight away from it (never forward:
        firmware 2026-11-05d allows only a backward drive while tilted; older firmware refuses every leg, so it
        holds), logged. Then the leg's point is aimed again around the keep-out (or skipped)."""
        import observe as OB
        kind = OB.trip_kind("%s %s" % (r.get("result"), r.get("info")))
        if kind not in ("tilt", "drop"):
            return
        end = r.get("end")
        cam = self.cam_pose(0.6) or cam0
        x, y, th = (cam[0], cam[1], cam[2]) if cam else ((end[0], end[1], end[2]) if end else (None, None, None))
        if x is None:
            raise RunEnd("the robot's %s stop tripped and the camera cannot see where: stopped" % kind)
        ko = OB.trip_keepout(x, y, th, name="%s trip %d" % (kind, len(getattr(self, "trips", []) or []) + 1))
        self.trips = list(getattr(self, "trips", []) or []) + [ko]
        self.S.keepouts.append({"name": ko["name"], "polygon": [(ko["center"][0] + ko["radius"] * math.cos(a_),
                                                              ko["center"][1] + ko["radius"] * math.sin(a_))
                                                             for a_ in [k_ * math.pi / 8 for k_ in range(16)]],
                                "kind": "trip"})
        say("SAFETY: %s's %s stop tripped (%s) at %.0f, %.0f: that spot is now kept out; backing straight away %.0f cm "
            "(never forward)" % (self.name, "tilt" if kind == "tilt" else "edge (drop)", r.get("info"), x, y,
                                 OB.BACK_OFF_CM))
        # (11-05h, 4') backing away is a reverse: over the leg it has just driven (its own trail: it came forwards
        # from cam0 along its heading, no camera needed), or with a fresh camera fix showing room behind; else hold
        cam_now = self.cam_pose(0.6)
        others = [tuple(np.array(k_["polygon"]).mean(0)) for k_ in self.S.keepouts if k_.get("kind") == "robot"]
        tr = OB.Trail()
        if cam0 is not None and r.get("start") and r.get("end") and r["result"] != "refused":
            came = math.hypot(r["end"][0] - r["start"][0], r["end"][1] - r["start"][1])
            for k_ in range(int(came) + 1):
                tr.add(x - (came - k_) * math.cos(math.radians(th)), y - (came - k_) * math.sin(math.radians(th)), th)
        how, L_, why_ = OB.back_out_plan(OB.BACK_OFF_CM, trail=tr, bubble_cm=1.0, cam=cam_now,
                                         cam_age_s=0.0 if cam_now else None, rect=(0.0, 0.0, self.S.L, self.S.W),
                                         others=others, tilted=True)
        ok_ = how in ("reverse_trail", "reverse_camera") and L_ >= 3.0
        if not ok_:
            self.ev("trip", stop_kind=kind, info=r.get("info"), at=[round(x, 1), round(y, 1), round(th, 1)], keepout=ko,
                    back_off="refused", back_info=why_)
            raise RunEnd("%s tripped its %s stop and backing away is not safe (%s): it stands where it is; nothing "
                         "else moves it" % (self.name, kind, why_))
        say("  backing away: %s" % why_)
        rb_ = self.rb.leg("go", -min(OB.BACK_OFF_CM, L_), timeout=15.0)
        self.odo_step(rb_)
        self.ev("trip", stop_kind=kind, info=r.get("info"), at=[round(x, 1), round(y, 1), round(th, 1)], keepout=ko,
                back_off=rb_.get("result"), back_info=rb_.get("info"))
        if rb_.get("result") != "done":
            raise RunEnd("%s tripped its %s stop and could not back away (%s %s): stopped where it is, nothing "
                         "else moves it" % (self.name, kind, rb_.get("result"), rb_.get("info")))
        say("  backed away %.0f cm; carrying on around the kept-out spot" % OB.BACK_OFF_CM)

    def watch_heading(self, row, value):
        """(11-05h) Observe, then trust, throughout: after every straight go leg of 8 cm or more, the camera's direction
        of motion against the tag's corrected heading. Two legs in a row the other way round (a tag knocked, or the
        start's check fooled) re-learn the offset, logged; the run carries on."""
        w = getattr(self, "hwatch", None)
        if w is None or value < 8.0 or row.get("cam0") is None or row.get("cam1") is None:
            return
        w.reset()
        w.feed(0, row["cam0"][:3], None, 1)
        ev = w.feed(1, row["cam1"][:3], None, 1)
        if not ev:
            return
        if ev["kind"] == "heading_bad":
            say("  HEADING: the leg moved %.0f deg off where the tag says the nose points (%d in a row)" % (
                ev["err_deg"], ev["in_a_row"]))
            self.ev("heading_bad", **ev)
        elif ev["kind"] == "heading_relearned":
            d = wrap180(ev["new_deg"] - ev["old_deg"])
            self.eye.yaw_off = dict(getattr(self.eye, "yaw_off", {}) or {})
            self.eye.yaw_off[self.tag] = wrap180(self.eye.yaw_off.get(self.tag, 0.0) + d)
            w.offset = 0.0
            say("  HEADING RE-LEARNED: two legs in a row moved %+.0f deg from the tag's heading: tag %d's offset is now "
                "%+.0f deg (observed); the run carries on" % (d, self.tag, self.eye.yaw_off[self.tag]))
            self.ev("heading_relearned", tag=self.tag, change_deg=d, offset_deg=self.eye.yaw_off[self.tag])
            import tag_heading_store as THS          # (e15) remembered for the next runs (the evidence wins)
            if THS.enabled(self.site):
                say("  " + THS.record(state_path(self.a, THS.FILE), self.tag, self.eye.yaw_off[self.tag],
                                      "the calibration's heading watch (re-learned)", robot=self.rb.name)["words"])

    def calibrate_one(self):
        """One robot (self.rb, self.name, self.tag): its values read, phases A, B (fit, set, save), C, park at its
        spot, `sleep 10`."""
        self.set_status(robot=self.name, tag=self.tag, wifi_name=self.rb.name, spot=self.S.park_label)
        self.before = {}
        for k in PARAMS:
            v = self.rb.get(k)
            if v is None:
                raise RunEnd("the robot did not answer `get %s`: stopped before anything changed" % k)
            self.before[k] = v
        with open(os.path.join(self.folder, "robot_values_before.json"), "w") as f:
            json.dump({"robot": self.name, "wifi_name": self.rb.name, "values": self.before,
                       "how": "read with get before anything was sent"}, f, indent=1)
        say("%s's current values (saved first): %s" % (self.name, self.before))
        self.baseline, words = self.baseline_mode()
        self.pooled, self.pool_needs = {}, None
        if POOL_ON():
            # (e14, Nick 16:40) every leg this robot drove here before, finished runs or not: the pooled values
            import calib_pool as CP
            wifi = (getattr(self, "wifi", {}) or {}).get(self.rb.name) or self.rb.name
            self.pooled = CP.fit(self.pool_dir(), wifi)
            say("POOLED CALIBRATION for %s: %s" % (wifi, CP.words(self.pooled)))
            if self.pooled and all(k in self.pooled for k in PARAMS) and all(
                    BOUNDS[k][0] <= self.pooled[k] <= BOUNDS[k][1] for k in PARAMS):
                self.baseline = "pooled"
                words = "the pooled values from %d earlier run%s here (finished or not)" % (
                    self.pooled.get("n_runs", 0), "" if self.pooled.get("n_runs") == 1 else "s")
                self.pool_needs = CP.needs(self.pooled)
                say("  this run measures only what is still uncertain: %s" % ", ".join(
                    k for k, v in self.pool_needs.items() if v) or "nothing (a check tour only)")
        self.baseline_words = words
        say("PHASE A STARTS FROM %s: %s" % ("DEFAULTS 1 / 1 / 0" if self.baseline == "defaults" else
                                            "THE POOLED VALUES" if self.baseline == "pooled" else "ITS SAVED VALUES",
                                            words))
        self.set_status(baseline=self.baseline, baseline_words=words)
        self.ev("baseline", mode=self.baseline, words=words, before=self.before)
        self.phase_a_values = dict(self.before)
        if self.baseline == "pooled":
            for k in PARAMS:
                ok, got = self.rb.set_checked(k, round(self.pooled[k], 4))
                if not ok:
                    for kk in PARAMS:
                        self.rb.set_checked(kk, self.before[kk])
                    raise RunEnd("%s did not take %s %.4f (read %s): its own values put back, nothing moved" % (
                        self.name, k, self.pooled[k], got))
            self.phase_a_values = {k: round(self.pooled[k], 4) for k in PARAMS}
            if self.pooled.get("offset") and not (self.pool_needs or {}).get("offset"):
                self.offset = tuple(self.pooled["offset"])
        if self.baseline == "defaults":
            for k in PARAMS:
                ok, got = self.rb.set_checked(k, BASELINE_DEFAULTS[k])
                if not ok:
                    for kk in PARAMS:
                        self.rb.set_checked(kk, self.before[kk])
                    raise RunEnd("%s did not take %s %.4f (read %s): its own values put back, nothing moved" % (
                        self.name, k, BASELINE_DEFAULTS[k], got))
            self.phase_a_values = dict(BASELINE_DEFAULTS)
        b0 = self.check_battery()
        self.status["battery_start"] = b0
        if b0 is not None and b0 < LOW_BATTERY_NOTE_PCT:
            # (e14) values measured on a weak battery: the report says so (DOE row: calibration against battery)
            self.status["low_battery_start"] = b0
            self.ev("low_battery_calibration", battery=b0, under=LOW_BATTERY_NOTE_PCT)
            say("NOTE: %s starts its calibration at %d %% battery (under %d %%): a LOW-BATTERY CALIBRATION; its values "
                "are taken on a weak battery and the report says so" % (self.name, b0, LOW_BATTERY_NOTE_PCT))
        cam = self.cam_pose_sure()      # (11-10f, school 13:25: refused 2 s after the wiggle in dim light, ~7 fps)
        while True:
            try:
                cam = self.heading_check(cam)
                break
            except RunEnd:
                if not self.dark_recent() or self.stop_flag.is_set():
                    raise
                self.wait_light("its heading check")       # (e14) the picture went dark during it: again once back
                cam = self.cam_pose_sure()
        self.reanchor(cam, "start")
        result = "finished"
        try:
            self.stage()
            self.phase_a()
            self.phase_b()
            self.phase_r()          # (11-10b) the curve phase (only for robots that will drive curves; --arc-phase)
            self.phase_c()
        except LowBattery as e:
            result = "low battery: %s" % e
            say("LOW BATTERY: %s. Series stopped; parking." % e)
        except RunEnd as e:
            result = "ended: %s" % e
            say("ENDED: %s" % e)
            if self.dark_recent() and not self.stop_flag.is_set():
                # (e14) never end and sleep while the picture is dark: wait for it, then park as usual
                self.wait_light("before parking")
        except BindError:
            raise
        self.pool_after()                  # (e14) this run's legs kept, finished or not; pooled values saved when sure
        if getattr(self, "fit", None) is None and self.phase_a_values != self.before and \
                not getattr(self, "pool_saved", False):
            # ended before phase B: the robot gets its own values back (the defaults were only for phase A)
            for k in PARAMS:
                self.rb.set_checked(k, self.before[k])
            say("  %s's own values put back (the run ended before the fit)" % self.name)
        self.result = result
        parked = None
        if self.stop_flag.is_set():
            say("Stopped by hand (Ctrl+C, `stop` or Mission Control's Stop): NOT driven back to park")
        elif "camera lost" not in str(result):
            try:
                parked = self.park("park")
            except (RunEnd, LowBattery) as e:
                say("Could not park: %s" % e)
        self.parked = parked
        try:
            self.status["battery_end"] = self.check_battery()
        except LowBattery:
            self.status["battery_end"] = self.battery[-1][1] if self.battery else None
        if getattr(self.a, "sleep_at_end", False):
            # the old way (kept, switched off): a wake restarts the robot and wipes values not saved on it
            ck = int((self.site or {}).get("sleep_checkin_min", 10))     # (11-05h) school: hourly check-in
            self.rb.link.send(self.rb.ip, "sleep %d" % ck)
            k = self.link.mark()
            slept = self.link.wait_for(lambda n, l: n == self.rb.name and "sleeping" in l, k, 4.0)
            self.slept = bool(slept)
            say("Sent `sleep %d` to %s (%s): it wakes for 20 s every %d min; robot_sleep.py wake any brings it back" % (
                ck, self.name, "it answered" if slept else "no answer seen", ck))
        else:
            # (11-05h, Observe then trust) it stays awake: waking from sleep restarts it, which wipes every value not
            # saved on it (firmware older than 2026-11-05a cannot save). Mission Control's idle sleeper puts it to sleep
            # after observe.IDLE_SLEEP_MIN minutes with no run
            self.slept = False
            say("%s stays awake here (its values stay in force); started from Mission Control's Start screen it is "
                "sent to sleep as soon as the run ends (e07, Advanced: sleep when done), else after %.0f minutes with "
                "no run, or by hand: python3 tools/pi/robot_sleep.py sleep" % (self.name, _idle_min()))
        self.ev("end", result=result, parked=parked, slept=self.slept)
        return result


WORKSPACE_ON = os.environ.get("MIRLT_CALIB_WORKSPACE", "1") != "0"   # (11-10b) switch: 0 = the typed table only
READMAP_SEED = os.environ.get("MIRLT_CALIB_READMAP", "1") != "0"   # (11-10b) switch: 0 = every spot taken as readable
                                                                    # until the run learns otherwise (before e12)


def setup_order_words(site, conf_dir=None):  # conf_dir: Mission Control's folder (tests)
    """(e14, Nick 16:40: the standard setup order for any surface) 1 the table's corners (the rectangle: 4 corners,
    pre-filled by the corner tags or the camera's Measure, confirmed by the person), 2 the clear space round each robot,
    3 each robot moves and is identified and named. Step 4 (each robot's path) is Mission Control's (redesign plan)."""
    t = (site or {}).get("table") or {}
    md = conf_dir or (site or {}).get("mc_dir") or ""
    ok = ""
    try:
        with open(os.path.join(md, "tag_setup.json")) as f:     # Mission Control's setup (tag_job: corners_ok)
            c = json.load(f).get("corners_ok") or {}
        if c.get("t"):
            ok = "confirmed %s" % time.strftime("%d %b %H:%M", time.localtime(c["t"]))
    except (OSError, ValueError):
        pass
    return ("SETUP ORDER: 1 the table's 4 corners (%.0f x %.0f cm; %s), 2 clear space round each robot, 3 each robot "
            "moves and is found and named" % (float(t.get("length_cm") or 0), float(t.get("width_cm") or 0),
                                              ok or "the camera's measure or the typed table, not confirmed on this "
                                              "computer: Mission Control, Setup, confirms the corner tags"))


def POOL_ON():
    """(e14) Calibration evidence pooled across runs (calib_pool.py); MIRLT_CALIB_POOL=0: each run on its own."""
    import calib_pool as CP
    return CP.enabled()


def OBS_READ_OK():
    import observe as OB
    return OB.READ_OK


def seed_readmap(rm, site, sim_cam=None, cal=None, dry=False, state_dir="", grey=None):
    """(11-10b, e12 item 2) The calibration's readability map: the camera model's guess (a tag reads where it is at least
    READ_PX (simulator) / observe.TAG_PX_MIN pixels in the picture at its worst heading), then earlier runs' reads at this
    site (readmap.json in the site's folder; a dry run's own sim folder), half weight. Returns words."""
    import observe as OB
    words = []
    tg = (site or {}).get("tags") or {}
    z, tcm = float(tg.get("height_cm") or 6.19), float(tg.get("tag_cm") or 8.0)
    if dry and sim_cam is not None:
        def fn(x, y):
            px = [sim_cam.tag_px(x, y, z, tcm, th) for th in range(0, 360, 45)]
            return 0.9 if all(p_ is not None and p_ >= READ_PX * 1.05 for p_ in px) else 0.1
        rm.seed(fn, "the simulated camera: a tag needs %.0f px at every heading" % READ_PX)
        words.append("seeded from the simulated camera")
    elif cal and os.path.exists(cal):
        import e02_camera as CAM
        m = CAM.load_camera(cal)
        OB.seed_from_cameras(rm, [(lambda P, m=m: [tuple(m.project(tuple(q))) for q in P], tuple(m.image_size))], tcm, z)
        words.append("seeded from the camera model (%s)" % os.path.basename(cal))
    folder = state_dir if dry else ((site or {}).get("conf_dir") or "")
    hp = os.path.join(folder, "readmap.json") if folder else ""
    import readmap_store as RS
    if RS.enabled() and folder:
        # (e14) the evidence for the light now and any tag (readmap_store): never the site's readmap.json merged at
        # full weight (10 Nov: a dim session shrank the area to 26 cm); MIRLT_READMAP_STORE=0 the old way below
        RS.import_old(folder, OB.ReadMap)
        words.append(RS.choose(folder, rm, grey, None))
        hp = ""
    if hp and os.path.exists(hp):
        with open(hp) as f:
            if rm.merge_history(OB.ReadMap.from_json(json.load(f))):
                words.append("earlier runs' reads here carried in (readmap.json, half weight)")
    bl = OB.blind_cells(rm)
    words.append("%d cm2 readable; %d spot%s learned blind" % (rm.area_cm2(), len(bl), "" if len(bl) == 1 else "s"))
    return "; ".join(words)


def save_readmap(rm, folder, site, dry=False, state_dir=""):
    """(11-10b) This run's readability map into the session, and into the site's folder for the next run's seed (a dry
    run: its own sim folder, never the Pi's), with blind_areas.json (the spots learned blind) beside it."""
    import observe as OB
    out = []
    dirs = [folder, state_dir if dry else ((site or {}).get("conf_dir") or "")]
    for d_ in dirs:
        if not d_:
            continue
        try:
            os.makedirs(d_, exist_ok=True)
            rm.save(os.path.join(d_, "readmap.json"))
            with open(os.path.join(d_, "blind_areas.json"), "w") as f:
                json.dump({"cells": OB.blind_cells(rm), "cell_cm": rm.cell, "how": "tries >= %d, read rate under %.0f %%" % (
                    OB.BLIND_MIN_TRIES, 100 * OB.READ_OK)}, f, indent=1)
            out.append(d_)
        except OSError:
            pass
    return out


def roam_motion_for(a, site):
    """(11-10b) How free roam drives at this site: (curves | stopturn, where that came from). --roam-motion (Mission
    Control passes its Advanced 'Free roam driving'), else Mission Control's saved Advanced for this site
    (<mc_dir>/advanced.json roam_motion), else the site file's roam_motion, else free roam's default (curves)."""
    m = (getattr(a, "roam_motion", "") or "").strip().lower()
    if m in ("curves", "stopturn"):
        return m, "as given to this run"
    md = (site or {}).get("mc_dir")
    if md:
        try:
            with open(os.path.join(md, "advanced.json")) as f:
                m = str(json.load(f).get("roam_motion") or "").lower()
            if m in ("curves", "stopturn"):
                return m, "Mission Control's Advanced 'Free roam driving'"
        except (OSError, ValueError):
            pass
    m = str((site or {}).get("roam_motion") or "").lower()
    if m in ("curves", "stopturn"):
        return m, "the site file's roam_motion"
    return "curves", "free roam's default"


def start_cpu_log(folder, procs, find):
    """(11-05h, CLAUDE.md heat tracker) cpu.csv in the session folder (e02_cpulog: each core, temperature, throttling,
    our processes), as the missions have; None when it cannot start (a laptop: no /proc or vcgencmd)."""
    try:
        import e02_cpulog as CL
        c = CL.CpuLog(folder, procs=procs, find=find)
        c.start()
        return c
    except Exception as e:  # noqa: BLE001
        say("(cpu log not started: %s)" % e)
        return None


def _idle_min():
    import observe as OB
    return OB.IDLE_SLEEP_MIN


def lead_value(value, speed, lead):
    """(11-05h) The go leg's length to send: the asked length less the robot's measured overrun at this speed (the
    nearest speed measured), so the camera sees it end where asked. None / no lead: as asked. Legs under twice the
    lead are sent as asked (a tiny leg's overrun is smaller)."""
    if not lead or value <= 0:
        return value
    k = min(lead, key=lambda v: abs(v - speed))
    d = lead[k]
    if value < 2.0 * abs(d) or abs(d) > LEAD_MAX_CM:
        return value
    return round(value - d, 2)


class RunEnd(Exception):
    pass


class SkipPoint(Exception):
    """(11-05h) a tour point the camera cannot be relied on to see: left out, the tour goes on from where it is."""

    def __init__(self, cam, why):
        super().__init__(why)
        self.cam, self.why = cam, why


class Deferred(SkipPoint):
    """(e14) A tour point whose tag was just lost on the way: moved to the end of the tour (once), not aimed at again
    at once."""


PARK_GIVEUP = __import__("safety_group").flag("MIRLT_PARK_GIVEUP")   # (e19, audit 8: through the Safety group)   # (e14) 1: parking ends at a lost tag (e13's way)
RETRY_LOST = os.environ.get("MIRLT_RETRY_LOST", "defer")   # (e14) "defer" (default) / "again" (before e14: up to 4)


class Recovered(Exception):
    """(11-05c) a leg inside goto() was interrupted and recovered: the robot is elsewhere now, so the rest of that
    goto's legs (planned from before) must not run: goto() aims again from the camera."""

    def __init__(self, cam):
        super().__init__("recovered")
        self.cam = cam


class LowBattery(Exception):
    pass


def _f(v):
    return "" if v == "" or v is None else "%.2f" % v


def _unwrap_turn(r):
    """(belief turn, dt, camera turn) unwrapped round the commanded turn (a +-180 turn's change is ambiguous by 360)."""
    v = r["value"]
    b = v + wrap180(r["bel1"][2] - (r["bel0"][2] + v))
    c = v + wrap180(r["cam1"][2] - (r["cam0"][2] + v))
    return (b, r["dt"], c)


def discover(link, ips, wake_min, dry=False, count=0, only=()):
    """{wifi name: ip} of the robots that answer `status` (sent to the given addresses and the broadcast; `wake` too, so
    a sleeping robot answers at its next check-in), for up to wake_min minutes. (5 Nov) A robot heard at all (its
    check-in, its hello) is also asked at the address it was heard from. count: keep going until this many robots have
    answered (0: the first to answer and any other within 2 s, as before)."""
    targets = list(ips) + ([] if dry else ["255.255.255.255"])
    end = time.time() + wake_min * 60
    found = {}
    k = link.mark()
    said = time.time()
    first = None
    while time.time() < end:
        with link.cond:
            heard = {n: ip for t, n, ip, line in link.lines[k:]}
            for t, n, ip, line in link.lines[k:]:
                if (line.startswith("M,status ") or ": awake" in line) and (not only or n in only):
                    found[n] = ip     # (11-05h) --only: other robots (on for flashing) are not taken
        if found and first is None:
            first = time.time()
        if only and set(only) <= set(found):
            return found
        if only:
            pass
        elif (count and len(found) >= count) or (not count and first is not None and time.time() - first >= 2.0):
            return found
        for ip in targets + [ip for n, ip in heard.items() if n not in found and ip not in targets]:
            link.send(ip, "wake")
            link.send(ip, "status")
        time.sleep(2.0)
        if time.time() - said > 30:
            said = time.time()
            say("  %s (a sleeping robot answers at its check-in, every 10 min): still trying, %.0f min left" % (
                "no robot answered yet" if not found else "%d of %d answered (%s)" % (len(found), count,
                                                                                    ", ".join(found)),
                (end - time.time()) / 60))
    return found


# =============================================================================================== the simulator
def sim_map(S, folder, names):
    import production as PR
    pts = {k: list(v) for k, v in S.points.items()}
    m, _w = PR.site_map((S.L, S.W), list(names) if isinstance(names, (list, tuple)) else [names], points=pts,
                        keepouts=S.keepouts)
    p = os.path.join(folder, "sim", "map.json")
    os.makedirs(os.path.dirname(p), exist_ok=True)
    PR.save_map(m, p)
    return p


def sim_xy(S, lab):
    """(e14) A simulated robot's start: a sticker or own spot by its label, "X,Y" in cm, else the run's first park spot
    (own spots) or the middle of the table."""
    if lab in S.points:
        return tuple(S.points[lab])
    try:
        x_, y_ = (float(v) for v in str(lab).split(","))
        return x_, y_
    except ValueError:
        pass
    if S.points:
        return tuple(S.points.get(getattr(S, "own_centre", None) or next(iter(S.points))))
    return S.L / 2.0, S.W / 2.0


def sim_camera_for(S, z_cm=145.0):
    """(e14) The simulator's one camera: the school's (behind the TV end, looking down the table) on the school's table;
    on any other table a camera 60 cm behind its x = 0 end, as high, looking at 45 % of its length (so a new room's dry
    run has a camera that sees its table). Returns (SchoolCamera, look_at)."""
    if abs(S.L - 396.2) < 5 and abs(S.W - 167.6) < 5:
        return SchoolCamera(z_cm=z_cm), (150.0, 83.8, 0.0)
    look = (0.45 * S.L, S.W / 2.0, 0.0)
    return SchoolCamera(z_cm=z_cm, behind_cm=60.0, y_cm=S.W / 2.0, look_at=look), look


def start_sim(a, S, folder):
    """fleet_sim.py: one robot named from its WiFi (not in the site), on P11 facing +y, ASLEEP, with the deliberately
    wrong true values (what it really needs: --true-cal), its fixes to SimEye."""
    name = a.sim_name
    mp = sim_map(S, folder, [name] + [q.split(":")[0] for q in a.sim_also])
    x, y = sim_xy(S, getattr(a, "sim_start", None) or "P11")
    starts = os.path.join(folder, "sim", "starts.json")
    st = {name: [x, y, float(getattr(a, "sim_start_heading", 90.0) or 90.0)]}
    for spec in a.sim_also:
        # (5 Nov) more robots: NAME:TAG:POINT[:HEADING], awake, standing on that sticker (e14: or X;Y in cm)
        q = spec.split(":")
        px, py = sim_xy(S, q[2].replace(";", ","))
        st[q[0]] = [px, py, float(q[3]) if len(q) > 3 else 90.0]
    with open(starts, "w") as f:
        json.dump(st, f)
    cmd = [PY, os.path.join(TOOLS, "sim", "fleet_sim.py"), "--map", mp, "--out", os.path.join(folder, "sim"),
           "--seed", str(a.seed), "--fixes-to", "127.0.0.1:%d" % (FIX_PORT + 100), "--fix-hz", "10", "--fix-sigma", "0",
           "--fix-sigma-deg", "0", "--fix-delay", "0.08", "--starts", starts, "--start-noise", "1.0",
           "--max-s", str(a.sim_max_s), "--drop-cm", "74", "--true-cal", "%s=%s" % (name, a.sim_true),
           "--asleep", "%s:%s" % (name, a.sim_sleep), "--quiet"]
    for spec in a.sim_also:
        q = spec.split(":")
        cmd += ["--true-cal", "%s=%s" % (q[0], a.sim_true_also or a.sim_true)]
    for fv in a.sim_fw:
        cmd += ["--fw", fv]
    for tp in getattr(a, "sim_tape", None) or []:
        cmd += ["--tape", tp]                  # (11-05h) a seam's dark tape the line sensors see
    if getattr(a, "sim_coast", None):
        for n_ in [name] + [q.split(":")[0] for q in a.sim_also]:
            cmd += ["--stop-coast", "%s=%s" % (n_, a.sim_coast)]
    for sv in getattr(a, "sim_saved", []) or []:
        cmd += ["--saved", sv]
    if getattr(a, "sim_arc_err", 0.0):
        cmd += ["--arc-err", "*=%g" % a.sim_arc_err]     # (11-10b) a deliberate curve error for the curve phase
    cmd += ["--persist"]       # (11-10b) the curve phase's follow runs must not end the simulator (it ends with the run)
    f = open(os.path.join(folder, "sim", "fleet_sim.txt"), "w")
    p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=REPO)
    time.sleep(3.0)
    return p, ["127.0.0.%d" % (2 + k) for k in range(1 + len(a.sim_also))]


# =============================================================================================== real camera
def start_bridge(a, site, folder):
    """The tag bridge on the recorder's pictures with the setup's camera model, every roof tag of the site (listed and
    spare ids) under the name tag<id>, its fixes to us."""
    try:                       # (11-10a, 9 Nov school) a camera reader left by an earlier run holds its port
        import stale_bridge as SB
        SB.clear((5011,), log=say)
    except Exception as e:     # noqa: BLE001  (never stops a run)
        say("(leftover camera reader check skipped: %s)" % e)
    tg = dict(site.get("tags") or {})
    ids = sorted(set(int(v) for v in (tg.get("ids") or {}).values()) | set(int(v) for v in tg.get("spares") or []))
    hs = [float(v) for v in (tg.get("heights_cm") or {}).values() if v]
    h = tg.get("height_cm") or (sorted(hs)[len(hs) // 2] if hs else None)
    tags = {"family": tg.get("family", "tag25h9"), "tag_cm": tg.get("tag_cm", 8.0), "roof_cm": tg.get("roof_cm", 9.5),
            "height_cm": h, "ids": {"tag%d" % i: i for i in ids}}
    tp = os.path.join(folder, "tags_bind.json")
    with open(tp, "w") as f:
        json.dump(tags, f, indent=1)
    tc = os.path.join(site.get("conf_dir") or os.path.expanduser("~/.config/mirlt"), "tagcal")
    cal = a.calib or os.path.join(tc, "overhead.json")
    if not os.path.exists(cal):
        raise RunEnd("no camera model at %s: run Measure first (Mission Control, Setup). Nothing moved." % cal)
    cmd = [PY, os.path.join(HERE, "tag_bridge.py"), "--tags", tp, "--cam", "overhead=%s,%s" % (a.stream, cal),
           "--out", os.path.join(folder, "bridge"), "--to", "127.0.0.1:%d" % FIX_PORT, "--wait-calib-s", "60"]
    tb = site.get("table") or {}
    if tb.get("length_cm") and tb.get("width_cm") and os.environ.get("MIRLT_CALIB_MOTION", "1") != "0":
        # (e13, school 10 Nov 12:12: the tag lost after "go 63 P11") the bridge's motion detector on: seeded by the tag
        # reads, it keeps following the robot where the tag stops reading; the hold uses its sightings to find it again
        cmd += ["--board", "0,0,%g,%g" % (float(tb["length_cm"]), float(tb["width_cm"]))]
    f = open(os.path.join(folder, "bridge.txt"), "w")
    return subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=REPO), cal, h


def cam_centre_from_model(path):
    try:
        from cameras import CameraModel
        m = CameraModel.load(path)
        return [float(v) for v in m.center()]
    except Exception:  # noqa: BLE001
        return None


# =============================================================================================== main
def session_dir(a):
    if a.session:
        return a.session
    return os.path.join(REPO, "logs", dt.datetime.now().strftime("%Y%m%d-%H%M%S") + "-calib" + ("-dry" if a.dry else ""))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--site", default="", help="the site (default: the one in use; tonight school-conference)")
    ap.add_argument("--robot", action="append", default=[], help="a robot's IP (also: found by the broadcast)")
    ap.add_argument("--wake-min", type=float, default=12.0, help="keep sending wake/status this long (sleep 10 = a "
                                                                  "check-in every 10 min)")
    ap.add_argument("--binding", choices=["wiggle", "fixed", "auto"], default="auto",
                    help="wiggle: bind the roof tag that follows each robot's wiggle (new); fixed: the site's tags.ids "
                         "(the old way); auto: the site's tags.binding, else wiggle")
    ap.add_argument("--lost-giveup", action="store_true",
                    help="(11-10g) the old way: a hold that does not see the tag for LOST_GIVEUP_S ends the run")
    ap.add_argument("--lost-policy", choices=["recover", "hold", "keep_going"], default=None,
                    help="recover (default, 5 Nov 16:30): the tag lost > %.1f s while driving -> stop, then slowly back "
                         "towards where it read well in short bubble-checked straight steps; hold: the old stop and "
                         "wait; keep_going: the experiments' policy (default: the site's calibration.lost_policy, else "
                         "recover)" % LOST_HOLD_S)
    ap.add_argument("--tours", type=int, default=TOURS_C, help="phase C tours (1-3)")
    ap.add_argument("--visit-order", "--tour-order", dest="tour_order", default="",
                    help="(11-10b) the tours' points in your order, e.g. P01,P05,P11 (as mission_run --visit-order): "
                         "phase A's tour and phase C's first tour in this order, then reversed, then shuffled")
    ap.add_argument("--arc-phase", choices=["auto", "on", "off"], default=None,
                    help="(11-10b) the curve phase after the fit: auto (default: the site's calibration.arc_phase, else "
                         "on for robots on firmware %s+ when free roam here drives curves), on, off" % ARC_FW)
    ap.add_argument("--roam-motion", default="", help="(11-10b) how free roam drives here (curves / stopturn), for "
                    "--arc-phase auto (default: Mission Control's Advanced for this site, else the site file, else curves)")
    ap.add_argument("--count", type=int, default=0, help="keep waking until this many robots answer (up to --wake-min; "
                                                         "0: the first to answer and any other within 2 s)")
    ap.add_argument("--no-save", action="store_true", help="do not save the fitted values on the robot (the site's "
                                                           "calibration.save_on_robot, default on)")
    ap.add_argument("--old-firmware-save", choices=["corr", "off", "auto"], default="auto",
                    help="a robot whose firmware has no `save` (before 2026-11-05a): corr = save with `corr save` "
                         "instead; off = not saved, said; auto = the site's calibration.old_firmware_save, else corr")
    bl = ap.add_mutually_exclusive_group()
    bl.add_argument("--baseline-defaults", dest="baseline", action="store_const", const="defaults", default="auto",
                    help="phase A starts from wheel_scale 1, gyro_scale 1, gyro_drift_dps 0 (an uncalibrated baseline) "
                         "even for a robot calibrated before (default: only its first calibration on this site)")
    bl.add_argument("--from-saved", dest="baseline", action="store_const", const="saved",
                    help="phase A starts from the robot's saved values even on its first calibration")
    ap.add_argument("--only", default="", help="calibrate only these robots, by WiFi name (e.g. AL-1DC0); any other robot "
                    "that answers (switched on for flashing) is left alone. Waits until each named one answers")
    ap.add_argument("--sim-start", default="", help="simulator: the sticker the robot starts on (default P11)")
    ap.add_argument("--view-x-max", type=float, default=None,
                    help="points and legs past this x (cm from the TV end) are skipped as out of the camera's view "
                         "(default %.0f). School 5 Nov: the roof tag read reliably to about x 100 cm in the room light "
                         "and the run ended losing it on the way to P11 (x 150): 110 keeps zone 1 (P01-P10)" % (
                             VIEW_X_MAX_CM - VIEW_MARGIN_CM))
    ap.add_argument("--park-spots", default="", help="spots in order, e.g. P09,P01,P05 (default: the site's "
                    "calibration.park_spots). School 5 Nov: P11 reads poorly in the room light, so start and park on P09")
    ap.add_argument("--sim-tape", action="append", default=[], metavar="X0,Y0,X1,Y1",
                    help="(simulator) a dark tape line on the table (the school seam's tape)")
    ap.add_argument("--sim-coast", default="", help="(simulator) every robot's wheels run down with this time "
                    "constant (s) after a brake: 0.08 reproduces AL-1DC0's overrun at school (5 Nov 19:34)")
    ap.add_argument("--sleep-at-end", action="store_true", default=None,
                    help="(the old way, kept) send `sleep 10` to each robot when it is parked. Default (11-05h): it "
                         "stays awake, so values not saved on it survive (the site's calibration.sleep_at_end)")
    ap.add_argument("--stop-lead", choices=["auto", "off"], default=None,
                    help="(11-05h) auto (default; the site's calibration.stop_lead): after the fit, each go leg is sent "
                         "short by the overrun phase A measured at its speed, so it ends on the spot; off: sent as asked")
    ap.add_argument("--safe-edge", type=int, default=None, choices=[0, 1, 2, 3],
                    help="the robot's own edge guard: 0 off, 1 line sensors, 2 the distance sensor's bottom zone (a real "
                         "drop), 3 both (default: the site's calibration.safe_edge, else 3). School 5 Nov: the line "
                         "sensors read the black tape over the table seam as an edge and refused the wiggle -> 2")
    ap.add_argument("--min-firmware", default=None, help="refuse a robot on firmware older than this (default: the "
                    "site's calibration.min_firmware, else %s; \"\" = no check)" % MIN_FIRMWARE)
    ap.add_argument("--no-pins", dest="pins", action="store_false", help="do not use or write the robot <-> tag pins "
                    "(robot_pins.json): a full wiggle for every robot, as before 5 Nov 11:00")
    ap.add_argument("--state-dir", default="", help="where robot_pins.json and calibrated_robots.json live (default: "
                    "the site's folder on the Pi; a dry run: its own sim folder)")
    ap.add_argument("--session", default="")
    ap.add_argument("--stream", default="127.0.0.1:8000")
    ap.add_argument("--calib", default="", help="the camera model (default: the site's tagcal/overhead.json)")
    ap.add_argument("--no-focus", action="store_true", help="do not set the camera's focus first")
    ap.add_argument("--no-camera-check", action="store_true", help="(11-10b) do not check the camera model against "
                    "the reference tags at the start (MIRLT_CAMERA_SELFCHECK=0 too)")
    ap.add_argument("--quick", action="store_true", help="simulator: fewer turns, a shorter still period, 1 tour of "
                                                          "6 points (code checks)")
    ap.add_argument("--spots", choices=["auto", "stickers", "own"], default=None,
                    help="(e14) where the calibration drives: auto (default; MIRLT_CALIB_SPOTS, the site's "
                         "calibration.spots): the typed stickers when present and proven, else spots of its own inside "
                         "the readable area; stickers; own")
    ap.add_argument("--clear-space", choices=["auto", "yes", "no"], default="auto",
                    help="(e14) the clear space round each robot for its finding stroke: yes (confirmed), no (each robot "
                         "wiggles on the spot instead), auto (a terminal asks; Mission Control's Calibrate sets "
                         "MIRLT_CLEAR_SPACE_OK=1 when ticked; a dry run: yes)")
    g = ap.add_argument_group("simulator")
    g.add_argument("--dry", action="store_true")
    g.add_argument("--sim-dark", action="append", default=[], metavar="X,Y,R[,FACTOR]",
                   help="(e14) simulator: a dark patch (no tag reads there; the pictures FACTOR as bright, default 0.35)")
    g.add_argument("--sim-extra-camera", action="store_true", help="(e14) simulator: one more camera, looking away")
    g.add_argument("--sim-blackout", action="append", default=[], metavar="T:DUR",
                   help="(e14) simulator: the picture goes black T s after the camera starts, for DUR s (no tag reads; "
                        "the bridge's dark / light_back messages)")
    g.add_argument("--sim-light", type=float, default=None, help="(e14) simulator: the overhead camera's mean grey "
                   "(school: 91 lit, 70 lights off): the readability evidence is chosen and kept for it")
    g.add_argument("--quick-full-tour", action="store_true", help="with --quick: phase A's turns, rolls and still period "
                   "short, but every tour the whole P01-P20 (the edges and the parked robots are what is checked)")
    g.add_argument("--seed", type=int, default=1)
    g.add_argument("--sim-arc-err", type=float, default=6.0, help="(11-10b) simulator: every robot really turns this "
                   "%% tighter on curves than its own pose shows (the curve phase must find it); 0 = none")
    g.add_argument("--sim-name", default="AL-1DC0")
    g.add_argument("--sim-tag", type=int, default=2)
    g.add_argument("--sim-also", action="append", default=[], metavar="NAME:TAG:POINT[:HEADING]",
                   help="simulator: another robot, awake, on that sticker (e.g. AL-2B7F:3:P14)")
    g.add_argument("--sim-true-also", default="", help="simulator: the other robots' true values (default --sim-true)")
    g.add_argument("--sim-fw", action="append", default=[], metavar="NAME=VERSION",
                   help="simulator: that robot runs older firmware (e.g. AL-1DC0=2026-10-29b: no save)")
    g.add_argument("--sim-true", default="0.97,1.03,0.3", help="what the simulated robot really needs: wheel_scale,"
                                                              "gyro_scale,gyro_drift_dps")
    g.add_argument("--sim-saved", action="append", default=[], metavar="NAME=WS,GS,GD",
                   help="simulator: that robot already has these values saved (loaded at power-on)")
    g.add_argument("--sim-sleep", default="1:0.4", help="the simulated robot sleeps MIN minutes, next check-in after "
                                                        "FRAC of it")
    g.add_argument("--cam-z-cm", type=float, default=145.0, help="the simulated camera's lens above the table top "
                   "(145: 7.5 ft above the floor; 228.6: 7.5 ft above the table)")
    g.add_argument("--no-height", action="store_true", help="simulator: the camera reads with NO height correction")
    g.add_argument("--sim-max-s", type=float, default=3600)
    g.add_argument("--sim-gap-at", action="append", default=[], metavar="X,Y,R:DUR",
                   help="simulator (11-05c): the tag unread while the robot is within R cm of X,Y, for DUR s from the first time")
    g.add_argument("--sim-gap", action="append", default=[], metavar="T:DUR",
                   help="simulator: the roof tag unread for DUR s from T s after the camera starts (a hand over it)")
    a = ap.parse_args(argv)
    import sites as SI
    if a.site:
        os.environ["MIRLT_SITE"] = a.site
    site = SI.load(a.site or None)
    cal_conf = site.get("calibration") or {}
    if not a.robot and not a.dry:
        a.robot = list(cal_conf.get("robot_ips") or [])     # the site's hint (school: Alvik 3), plus the broadcast
    if a.binding == "auto":
        a.binding = ((site.get("tags") or {}).get("binding") or "wiggle")
    if a.dry and not a.count:
        a.count = 1 + len(a.sim_also)
    a.save_on_robot = (not a.no_save) and bool(cal_conf.get("save_on_robot", True))
    if a.old_firmware_save == "auto":
        a.old_firmware_save = str(cal_conf.get("old_firmware_save") or "corr")
    if a.safe_edge is None:
        a.safe_edge = int(cal_conf.get("safe_edge", 3))
    if a.min_firmware is None:
        a.min_firmware = cal_conf.get("min_firmware", MIN_FIRMWARE) or ""
    if a.baseline == "auto":
        a.baseline = str(cal_conf.get("baseline") or "auto")
        a.baseline = a.baseline if a.baseline in ("auto", "defaults", "saved") else "auto"
    a.park_spots_given = bool(a.park_spots or cal_conf.get("park_spots"))
    a.park_spots = tuple(x.strip() for x in a.park_spots.split(",") if x.strip()) if a.park_spots else \
        tuple(cal_conf.get("park_spots") or PARK_SPOTS)
    if a.sleep_at_end is None:     # (11-05h) the site's calibration.sleep_at_end, else stay awake
        a.sleep_at_end = bool(cal_conf.get("sleep_at_end", False))
    if a.stop_lead is None:        # (11-05h) the site's calibration.stop_lead, else auto
        a.stop_lead = str(cal_conf.get("stop_lead") or "auto")
    if a.arc_phase is None:        # (11-10b) the site's calibration.arc_phase, else auto
        a.arc_phase = str(cal_conf.get("arc_phase") or "auto").lower()
        a.arc_phase = a.arc_phase if a.arc_phase in ("auto", "on", "off") else "auto"
    if a.lost_policy is None:      # (11-05c) the site's calibration.lost_policy, else recover
        a.lost_policy = str(cal_conf.get("lost_policy") or "recover")
    a.pins = a.pins and bool((site.get("tags") or {}).get("pin", True))
    if a.quick:
        global TURNS_A, ROLLS_A, STILL_S, STILL_C_S
        TURNS_A, ROLLS_A, STILL_S, STILL_C_S = (90, -90, 170, -170), (20.0, 40.0), 20.0, 10.0
    folder = session_dir(a)
    os.makedirs(folder, exist_ok=True)
    a.state_dir = a.state_dir or (os.path.join(folder, "sim") if a.dry else (site.get("conf_dir") or ""))
    say("Calibration run %s, session %s" % (VERSION, folder))
    say(setup_order_words(site))      # (e14, Nick 16:40) corners -> clear space -> robots found and named
    try:
        import engine_checks as EC_
        say("BATTERY FLOOR: %s" % EC_.battery_floor_used(site=site)[1])      # (e14) the floor used, and where from
    except Exception as e:  # noqa: BLE001
        say("(battery floor not read: %s)" % e)
    say("Firmware: %s. Pins: %s. Baseline: %s. Spots: %s." % (
        "robots older than %s are refused" % a.min_firmware if a.min_firmware else "not checked",
        state_path(a, PINS_FILE) if a.pins else "off (a full wiggle for every robot)",
        {"auto": "defaults on a robot's first calibration here, else its saved values",
         "defaults": "defaults for every robot (asked)", "saved": "saved values for every robot (asked)"}[a.baseline],
        ", ".join(a.park_spots)))
    run = CalibRun(a, site, folder)
    if a.quick and not a.quick_full_tour and run.S.spots_mode == "stickers":
        run.S_quick = True
        z = run.S.zone12()
        run.S.zone12 = lambda: ["P10", "P14", "P17", "P12", "P20", "P13"] if z else z
    if a.tour_order:
        # (11-10b) points in your order: every tour uses them (unknown names refused before anything moves)
        order = [q.strip().upper() for q in a.tour_order.replace(";", ",").replace(" ", ",").split(",") if q.strip()]
        bad = [q for q in order if q not in run.S.points]
        if bad:
            raise SystemExit("--visit-order: no such point here: %s (points: %s). Nothing moved." % (
                ", ".join(bad), ", ".join(sorted(run.S.points))))
        run.S_quick = True                    # the robots of a multi-robot run take the same order (run() copies it)
        run.S.zone12 = lambda: list(order)
        say("TOURS IN YOUR ORDER: %s" % " ".join(order))
    procs = []

    def stop(*_):
        run.stop_flag.set()
        stop_every_robot(run)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    cam_c, tag_h = None, None
    cpu = start_cpu_log(folder, {"calib": os.getpid()}, {"bridge": "tag_bridge.py", "recorder": "camrec.py"})
    if (site or {}).get("attended_only") and not a.dry:
        say("ATTENDED ONLY (Jay-Sun 5 Nov): at school the robots drive only while Jay-Sun is in the room")
    def own_spots():
        # (e14) no proven stickers: the run makes its own spots inside the readable area the camera gives it
        if run.S.spots_mode != "own":
            say("SPOTS: %s" % run.S.spots_why)
            return
        w_ = run.S.make_own_spots(lambda x_, y_: run.readmap().readable(x_, y_)[0])
        say("SPOTS: " + w_)
        if not run.S.points:
            raise RunEnd("the calibration found no spot of its own to drive to: %s. Nothing moved" % w_)
        if not (getattr(a, "park_spots_given", False)) or not all(q in run.S.points for q in a.park_spots):
            a.park_spots = run.S.own_park_spots()
        run.S.park_label = a.park_spots[0]
        say("  its park spots: %s; the tours: %s" % (", ".join("%s (%.0f, %.0f)" % ((q,) + tuple(run.S.points[q]))
                                                                 for q in a.park_spots), " ".join(run.S.zone12())))
        if a.quick and not a.quick_full_tour:
            run.S_quick = True
            z_ = run.S.zone12()[:6]
            run.S.zone12 = lambda: list(z_)
        run.ev("own_spots", words=w_, spots=run.S.points, park=list(a.park_spots))
    try:
        if a.dry:
            loopback_up()
            cam, look_ = sim_camera_for(run.S, a.cam_z_cm)
            a.sim_look = look_
            run.sim_cam = cam
            own_spots()
            sp, ips = start_sim(a, run.S, folder)
            procs.append(sp)
            a.robot = a.robot or ips
            cam_c = cam.C.tolist()
            tag_h = 6.19
            run.sim_cam = cam
            run.sim_true = dict(zip(PARAMS, (float(v) for v in a.sim_true.split(","))))
            run.sim_true_by = {a.sim_name: run.sim_true}
            for q in a.sim_also:
                run.sim_true_by[q.split(":")[0]] = dict(zip(PARAMS, (float(v) for v in (a.sim_true_also or
                                                                                         a.sim_true).split(","))))
            simtags = {a.sim_name: a.sim_tag}
            simtags.update({q.split(":")[0]: (-1 if q.split(":")[1] in ("-", "none") else int(q.split(":")[1]))
                            for q in a.sim_also})
            run.sim_tags = simtags
            def eye_factory():
                eye = SimEye(folder, FIX_PORT + 100, cam, simtags, tag_h=tag_h, correct=not a.no_height,
                             seed=a.seed, gaps=a.sim_gap)
                eye.darks = []
                for bo in a.sim_blackout:        # (e14) a sudden blackout that recovers
                    t_, _, d_ = bo.partition(":")
                    eye.blackout(float(t_), float(d_))
                for d_ in a.sim_dark:            # (e14) dark patches
                    v_ = [float(q) for q in d_.split(",")]
                    eye.darks.append((v_[0], v_[1], v_[2], v_[3] if len(v_) > 3 else 0.35))
                for g in a.sim_gap_at:          # (11-05c) position-triggered tag dropouts
                    xyr, _, dur = g.partition(":")
                    gx, gy, gr = (float(v) for v in xyr.split(","))
                    eye.gaps_at = getattr(eye, "gaps_at", []) + [dict(x=gx, y=gy, r=gr, dur=float(dur), t=None)]
                return eye
        else:
            if not a.no_focus:
                set_focus()
            if not a.no_camera_check:
                # (11-10b, e12) the camera model checked against the reference tags first; refitted when it disagrees
                try:
                    import camera_selfcheck as CSC
                    CSC.run(site, out=folder, log=say)
                except Exception as e:  # noqa: BLE001  (never stops a run: the model it has is used)
                    say("(camera self-check skipped: %s)" % e)
            bp, cal, tag_h = start_bridge(a, site, folder)
            a.cal_used = cal                       # (11-10b) the readability seed's camera model
            procs.append(bp)
            own_spots()                            # (e14) its own spots when the site has no proven stickers
            cam_c = cam_centre_from_model(cal)
            eye_factory = lambda: Eye(folder, FIX_PORT)    # noqa: E731
            import camera_feed as CF     # (e13) every camera sending new pictures, before anything binds or moves
            ok_, words_ = CF.check(os.path.join(folder, "bridge"))[:2]
            say(("CAMERA CHECK: " if ok_ else "NOT STARTED: ") + words_)
            if not ok_:
                raise RunEnd(words_)
        run.cam_centre, run.tag_h = cam_c, tag_h
        run.set_status(cam_centre=cam_c, tag_h=tag_h, sim_true=getattr(run, "sim_true", None))
        result = run.run(eye_factory)
    except (RunEnd, BindError) as e:
        result = "refused: %s" % e
        say("STOPPED: %s" % e)
        run.result = result
    finally:
        # (e19, audit H1) stop FIRST at every ending (an error too, not only Ctrl+C / SIGTERM): before e19 an unexpected
        # error waited up to 10 s per child process, then closed the link without a stop
        stop_every_robot(run)
        if cpu is not None:
            try:
                cpu.stop()
            except Exception:  # noqa: BLE001
                pass
        for p in procs:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        for p in procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
        for c in ("eye", "link"):
            o = getattr(run, c, None)
            if o:
                o.close()
        for f in (run.legs_f, run.raw_f, run.cor_f, run.events):
            f.close()
        if getattr(run, "rm", None) is not None:
            # (11-10b, e12 item 2) where the camera read the roof tag in this run: kept for the next run's seed
            kept = save_readmap(run.rm, folder, site, dry=bool(a.dry), state_dir=a.state_dir)
            for tg_, m_ in (getattr(run, "rm_tags", None) or {}).items():   # (e13) each tag's own evidence
                for d_ in ([folder] + ([a.state_dir] if a.dry else [(site or {}).get("conf_dir") or ""])):
                    if d_:
                        try:
                            m_.save(os.path.join(d_, "readmap_tag%s.json" % tg_))
                        except OSError:
                            pass
            if kept[1:]:
                say("Readability kept for the next run: %s" % os.path.join(kept[-1], "readmap.json"))
            try:
                # (e14) this run's own evidence per tag, kept as sessions with the light (readmap_store)
                import readmap_store as RS
                sd = a.state_dir if a.dry else ((site or {}).get("conf_dir") or "")
                if RS.enabled() and sd:
                    g_ = float(a.sim_light) if getattr(a, "sim_light", None) is not None else RS.grey_of(folder)
                    tags_ = getattr(run, "rm_tags", None) or {}
                    for tg_, m_ in tags_.items():
                        say("READABILITY for tag %s: %s" % (tg_, RS.add_session(
                            sd, m_, RS.evidence(m_, (getattr(run, "rm_tag_base", {}) or {}).get(tg_)), grey=g_,
                            tag=tg_, kind="calibration", sid=os.path.basename(folder))))
                    if not tags_:
                        say("READABILITY: %s" % RS.add_session(sd, run.rm, RS.evidence(run.rm, getattr(
                            run, "rm_base", None)), grey=g_, tag="*", kind="calibration", sid=os.path.basename(folder)))
            except Exception as e:  # noqa: BLE001
                say("(readability store not added to: %s)" % e)
    import calib_report as CR
    try:
        words = CR.write_multi(folder, run, a) if getattr(run, "subs", None) else CR.write(folder, run, a)
    except Exception as e:  # noqa: BLE001
        import traceback
        traceback.print_exc()
        words = "the report could not be written: %s" % e
    run.set_status(phase="done", result=result, words=words, ended=time.time(), closest=run.closest)
    say("RESULT: " + words)
    say("Report: %s" % os.path.join(folder, "report.md"))
    return 0 if result == "finished" else 1


def stop_every_robot(run):
    """(e19, audit H1) `stop` to every robot the run knows, each send on its own (a failing one cannot keep the others
    from it). Used first at every ending and by the Ctrl+C / SIGTERM handler."""
    n = 0
    for rb in list(getattr(run, "robots", None) or []):
        try:
            rb.link.send(rb.ip, "stop")
            n += 1
        except Exception:  # noqa: BLE001
            pass
    return n


def set_focus(dev="/dev/video0"):
    """The C920e's focus: autofocus off, focus_absolute 0 (both reset when it is unplugged or the Pi restarts)."""
    for ctrl in ("focus_automatic_continuous=0", "focus_auto=0", "focus_absolute=0"):
        try:
            subprocess.run(["v4l2-ctl", "-d", dev, "--set-ctrl=%s" % ctrl], capture_output=True, timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            say("(v4l2-ctl not found: the camera's focus was not set. ./run-fleet-pi.sh school-setup installs it)")
            return False
    return True


if __name__ == "__main__":
    import idle_after_run as IA     # (e13) robots asleep 10 min after the run ends if no new run starts
    with IA.track("calib"):
        _rc = main()
    sys.exit(_rc)
