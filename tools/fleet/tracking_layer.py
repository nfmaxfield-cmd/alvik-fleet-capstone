#!/usr/bin/env python3
"""ONE tracking layer (e09, Phase B1, 7 Nov; Nick: one pipeline, whatever the site): every camera source a site has
goes into the same layer, which says which robot each sighting is, and hands out named fixes (with the source and its
sigma) to the robot's filter (fused_pose.py) and the planner. Used by free roam (home) and Mission Control (school),
by the Pi's bridges (cam_bridge.py, tag_bridge.py) and by the fast simulator (freeroam_fastsim.py --camera detections).

Sources (each a switch; SOURCES_BY_TRACKING gives a site's defaults):
  tags     roof AprilTags: the tag's id names the robot (school). Read only where the picture is good enough (about
           x < 100 cm of the school table, room light)
  motion   the motion detector (floor memory, e02_multitrack): an outline, NO name; it does not see a robot that stands
           still, and loses robots on dark patches and against a busy background
  lock     the lock-on tracker (e02_lock): seeded on ONE robot (by a tag read, or by a sure motion sighting), it follows
           that robot's own look picture by picture, standing or moving. 22-23 Oct evening losses: seen in 94 / 93 / 96 %
           of pictures against 41 / 40 / 72 % for the detector alone (tests/test_e02_realclips.py). Its name is the
           robot it was seeded on, so a lock can carry a WRONG name if it jumped to a look-alike: it is checked like an
           unnamed sighting whenever two robots are close
  lights   the robots' lights (brightest spot near the prediction; home): no name
  ml       the ML detector (tools/detector, the MIRLT model): a slot, switched OFF (MIRLT_ML_DETECTOR=1 turns it on once
           the model's real accuracy is measured); handled as an unnamed sighting

Who is who ("identity"):
  - a tag names its robot; everything else is matched to the robots' predictions (their filter's pose, or their own
    pose) inside a gate, the least total distance; a sighting about as near two predictions (CONTESTED_CM), or one
    outline as big as two (merged), names NOBODY (unsure is flagged, never guessed: the 27 Oct swap was a guess);
  - WIGGLE FIRST (identity="wiggle", the default): two robots that came within CLOSE_CM of each other are a "close
    pass". When they part, each camera track's motion since the pass is compared with each robot's OWN motion (its
    wheels and gyro): kept or swapped, whichever explains it clearly better (SWAP_MARGIN_CM). When neither does (both
    stood, or moved alike) the layer asks for a WIGGLE of one of them (a quarter turn and back on its spot): the outline
    that turns in place without moving is that robot. Until then neither robot takes a camera fix (it drives on its
    filter, its uncertainty growing). Four swaps on the real table (22 Oct Right for Left; 27 Oct Left with a parked
    Right; 30 Oct lights matched twice; 5 Nov the camera put Center on Right's spot) were each fixed piecewise before;
  - identity="predict" (the switch's other side): nearest prediction only, as cam_bridge did before e09.

Also here (shared by both sites):
  - seeding: a tag read seeds the motion detector and the lock-on on THAT robot, so where tags stop reading (the far
    half of the school table) the robot is still followed, by name;
  - the parked-robot watch: a robot whose own wheels stand still while the camera sees it move PARKED_MOVE_CM is
    "moved while parked" (pushed by another robot, a hand): its pose is taken from the camera (evidence wins), the
    moment is logged and a picture asked for;
  - the brick ("what changed") watch: a sighting no robot can claim that stays put BRICK_S is a new obstacle (a hand
    passing is not: it moves or goes); the image-level watch (brickwatch.py) does the same on the bridges' pictures.

Numbers: every number below says whether it was measured or chosen; the chosen ones are listed in the e09 report's
Sweep 2 with whether a DOE should set them.
"""
import itertools
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import safety_group as SG     # noqa: E402  (e18: the safety switches' defaults in one place)

VERSION = "tracking-layer-2026-11-12e21"   # e21 (e20's open problem): the whole-camera verdict is made per camera (MIRLT_CAMERA_VERDICT per_camera): each named camera's sightings are watched on their own, and only the camera(s) the evidence implicates are refused (their sightings left out before assignment, so another camera's sighting of the robot is used); the run's searched robots widen their gate to their error shape (set_focus) and ask the detector. Before: "tracking-layer-2026-11-12e20"   # e20 (camera trust): every sighting carries its camera; each camera is checked against the robots' own motion, other cameras (a consensus) and the whole picture (camera_trust.TrustBook); a camera is refused where its own evidence says it is wrong; e19's whole-camera verdict still stands for a camera-wide fault, also counted as evidence, and a wrong-way sighting lying on another robot no longer counts toward it (its false alarm at 181 s, 11 Nov profile seed 1); a close pair that both stand apart is parted (MIRLT_PAIR_STAND_PART). MIRLT_CAMERA_TRUST=e19: the e19 layer. Before: "tracking-layer-2026-11-12e19"   # e19 (the Fable audit): the whole-camera watch (MIRLT_WHOLE_CAMERA_WATCH: most sightings outside every gate for a window -> the camera not trusted, said); an elimination beyond the robot's stated doubt needs motion proof (proof_fn, MIRLT_MOTION_PROOF). Before: "tracking-layer-2026-11-12e18b"   # e18b (the coordinator's replay of all 10 home runs, 5 Nov 14:55): the elimination counts robots in an unresolved close pass as unaccounted and refuses a spot on another robot's own pose (MIRLT_ELIM_PAIR_GUARD). e18 (Nick 12 Nov, decision 1): when two or more robots are unaccounted and a wiggle cannot tell their candidate sightings apart (none turned, both turned, or no turn on the spot is safe there), ONE robot at a time makes a short slow straight out-and-back (its length from the candidates' measured camera noise, 2-4 cm; its speed from the picture rate): the sighting that went out and came back is it (MIRLT_OUTBACK_ID=0: wiggles only, as e17). The e17 switches read through the Safety group. Before: "tracking-layer-2026-11-12e17"   # e17 (home 11 Nov: Left held 84 min 48 cm from where the camera saw it standing): observe, then trust. (a) a STEADY sighting no other robot can claim, with exactly one standing robot unaccounted for, is that robot however far from its own pose (assigned by elimination, flagged how="elimination", then on probation: revoked if it stays when the robot drives); (b) two or more unaccounted, or two unclaimed steady sightings: a wiggle of one robot, the sighting that turns (spin flag, heading change or scatter past its own standing noise) is it; (c) the gate grows with each robot's MEASURED own-vs-camera record (drift per cm, recent correction sizes; the board caps it), the e09 numbers stay as the starting guess; (4) the ML detector on demand (off: no model yet). Switches MIRLT_ELIMINATE, MIRLT_GATE_RECORD, MIRLT_ML_ON_DEMAND
VERSION_D = "tracking-layer-2026-11-07d"   # d (Nick 7 Nov): the camera watch ON by default (e10 method: 0 false alarms in 48 rehearsal runs); MIRLT_CAMERA_WATCH=0 turns it off
VERSION_C = "tracking-layer-2026-11-07c"   # c (e10): camera watch rebuilt (averaged checkpoints, noise-aware votes)
# b: camera watch off by default (Nick 7 Nov: off until its false alarms are understood)

SOURCES_BY_TRACKING = {"lights": ("motion", "lock", "lights"), "tags": ("tags", "motion", "lock"),
                       "none": ()}
OLD_SOURCES = {"lights": ("motion", "lights"), "tags": ("tags",), "none": ()}     # before e09 (the switch)
ML_DETECTOR = os.environ.get("MIRLT_ML_DETECTOR", "0") == "1"     # the ML slot: OFF until measured
IDENTITY = SG.value("MIRLT_IDENTITY")              # "wiggle" (e09) | "predict" (before)
CAMERA_WATCH = SG.flag("MIRLT_CAMERA_WATCH")    # the camera watch (turn directions); ON by default (Nick 7 Nov, after e10's fix: 0 false alarms in 48 runs); 0: off (e09b had it off)

GATE_CM = 6.0            # as e02_multitrack.GATE_CM: a sighting this near a prediction can be that robot ...
GATE_GROW_CMS = 10.0     # ... plus this much per second since it was last seen for sure (e02_multitrack)
GATE_PER_CM = 0.15       # CHOSEN (e09): the gate grows 0.15 cm per cm the robot's own wheels drove since it was sure
GATE_ROOT_CM = 2.0       # ... and 2 cm x sqrt(seconds) (a robot pushed, a slow drift); DOE: yes, with GATE_MAX_CM
GATE_MAX_CM = 25.0       # CHOSEN: e02_multitrack's 18 cm, widened for the filter's longer blind stretches
CONTESTED_CM = 2.5       # as e02_multitrack.AMBIG_CM: two explanations this close: nobody is named
MERGE_AREA = 1.55        # as e02_multitrack.MERGE_AREA: an outline this many robot areas big is two robots merged
CLOSE_CM = 16.0          # CHOSEN: middles this close = a close pass (bodies within ~6.5 cm: where the 27 Oct and
                         # 5 Nov swaps happened); DOE: yes, against swaps caught vs wiggles asked per hour
PART_CM = 22.0           # CHOSEN: ... and parted again past this (hysteresis)
SWAP_WIN_S = 2.5         # CHOSEN: the motion since the pass compared over at most this long
SWAP_MARGIN_CM = 2.0     # CHOSEN: one explanation must beat the other by this (summed displacement error)
MIN_MOTION_CM = 3.0      # CHOSEN: a track that moved less than this cannot show who it is by moving: wiggle
WIGGLE_S = 2.5           # the wiggle's watch window (a quarter turn and back at 92 deg/s: ~2 s, as wiggle_bind's)
WIGGLE_STILL_CM = 1.5    # CHOSEN: the wiggling robot's outline stays within this of its spot
WIGGLE_NEAR_CM = 12.0    # CHOSEN: only outlines turning within this of either robot's spot count for the wiggle
WIGGLE_CLEAR = 0.5       # CHOSEN: the turning outline must be at most half as far from one robot as from the other
WIGGLE_RETRY_S = 6.0     # a wiggle that showed nothing is asked again after this
NO_WIGGLE_WAIT_S = 8.0   # CHOSEN: with no wiggle possible, how long the pair waits for a tag read or clear motion
PARKED_STILL_S = 2.0     # CHOSEN: wheels standing this long = parked (for the parked watch)
PARKED_MOVE_CM = 3.0     # CHOSEN: the camera's spot moved this far while parked = moved (3 camera sigmas)
PARKED_MAX_CM = 15.0     # CHOSEN: ... and at most this far (a push or a hand's nudge; a jump further is a misread or
                         # another robot, never a reason to move a robot's pose)
PARKED_EVERY_S = 10.0    # CHOSEN: at most one parked-move a robot this often
PARKED_REF_N = 10        # CHOSEN: sightings averaged for where it was parked ...
PARKED_NOW_N = 6         # ... and for where it is now
BRICK_S = 2.0            # CHOSEN: an unclaimed sighting standing BRICK_S = a new object (brickwatch: 3 checks at 4 Hz
                         # = 0.75 s after the hand leaves; here the fast simulator's sightings, 12 a second)
BRICK_STILL_CM = 3.0     # as brickwatch.STILL_CM
BRICK_R_CM = 6.0         # CHOSEN: the keep-out round a confirmed object (a brick's half width + a margin)
OWN_RESETS = os.environ.get("MIRLT_OWN_RESETS", "1") != "0"   # (e10) 0: own-pose re-sets read as motion (as e09)
OWN_JUMP_CM = 5.0        # (e10) CHOSEN: an own pose step over this ...
OWN_MAX_CMS = 30.0       # ... plus 30 cm/s x its time (top speed 12.5 cm/s, x 2.4) is a re-set, not motion
OWN_RESET_RULE = SG.value("MIRLT_OWN_RESET_RULE")   # (e17) "velocity": a step off where its own
                         # recent velocity carries it by over OWN_JUMP_CM (+ the speed ramp) is a re-set, at any picture
                         # rate; "speed": the e10 rule (OWN_JUMP_CM + 30 cm/s x dt: at the home Pi's 4-5 pictures a second
                         # it read 25 of 39 re-seeds of the 08:26 run as drives, 11-12 cm a step: the dropped sightings)
OWN_ACCEL = 20.0         # (e17) firmware drive_accel (cm/s per s): how much its speed can change in a step. Not chosen
BRICK_NOT_NEAR_OWN_CM = 12.0   # (e10) CHOSEN: no new object within this of a robot's own pose (a robot's half diagonal +
                         # its own pose's drift); a brick put down right beside a robot is found once the robot moves on
DUPLICATE_RATIO = 0.33   # (e10) CHOSEN: a sighting 3 x nearer another robot's prediction (in its gate) is that robot's
CLUSTER_CM = 3.5         # CHOSEN: sightings of different sources this close are ONE object (a robot's outline, its lights
                         # and its lock), matched as one: one robot's lights can then not be handed to another robot whose
                         # gate has grown (30 Oct: lights matched to the wrong robot twice). DOE: no, under a body's half
LOCK_SEED_STILL_CM = 1.0     # as e02_camera RELOCK_STILL_CM: re-lock on a robot the detector sees standing still
CAM_CHECK_CM = 6.0       # CHOSEN: a camera checkpoint every 6 cm of a robot's camera travel
CAM_TURN_DEG = 20.0      # CHOSEN: over three checkpoints its own heading turned at least this much: a vote
CAM_CHECK_N = 6          # CHOSEN: the last 6 votes (any robots) ...
CAM_BAD_N = 4            # ... 4 of them the other way: the picture is not trusted (one robot's slip or swap is not enough)
CAM_GOOD_N = 4           # CHOSEN: 4 agreeing in a row: trusted again
CAM_WATCH_METHOD = os.environ.get("MIRLT_CAMERA_WATCH_METHOD", "e10")   # (e10) "e09": the old single-sighting check
CAM_AVG_S = 0.4          # (e10) CHOSEN: a checkpoint averages this long of sightings (~5 at 12 a second)
CAM_AVG_MIN = 3          # (e10) CHOSEN: ... at least this many
CAM_CHECK2_CM = 12.0     # (e10) CHOSEN: checkpoints this far apart (the chord turn's noise falls as 1 / length)
CAM_WIN_S = 6.0          # (e10) CHOSEN: three checkpoints within this long, or they start again
CAM_MIN_TURN_DEG = 8.0   # (e10) CHOSEN: an expected chord turn under this is no evidence either way; DOE: yes, on logs,
                         # with CAM_CHECK2_CM (false alarms per hour vs time to catch a mirrored camera)
SOURCE_SIGMA = {"tags": 0.6, "motion": 1.0, "lock": 1.2, "lights": 1.5, "ml": 1.5}   # see fused_pose.SRC_SIGMA

# (e17) observe, then trust: what the camera sees standing still wins over a robot's own drifted pose
ELIMINATE = SG.flag("MIRLT_ELIMINATE")        # (a, b) assign by elimination; 0: as e16 (gate only)
GATE_RECORD = SG.flag("MIRLT_GATE_RECORD")    # (c) the gate from each robot's measured record
# (e18, Nick 12 Nov decision 1) the out-and-back identity drive, after a wiggle that could not tell
OUTBACK_ID = SG.flag("MIRLT_OUTBACK_ID")
# (e18, the coordinator's replay of all 10 home runs: 5 Nov 14:55, Right corrected onto Left's spot 56 cm away)
WHOLE_CAMERA_WATCH = SG.flag("MIRLT_WHOLE_CAMERA_WATCH")   # (e19, audit 10) most sightings outside EVERY robot's gate
                        # for a window of pictures: the camera as a whole is not trusted (a wrong name, a bumped camera)
WHOLE_MOVE_CM = 3.0     # (e19) a robot "drove" / a sighting "moved" over half the window: 3 camera sigmas of the
                        # lights' 1 cm. Derived from the measured noise; DOE: no
WHOLE_TRANSFORM_DEG = 20.0   # (e19) robots' wrong-way motions share one mirror axis / one turn within this. Chosen
WHOLE_TURN_DEG = 20.0        # (e19) a turned camera: by more than this. Chosen; DOE: yes, both (bumped-camera faults
                             # of 5-45 deg in the hard simulator against the real logs' heading errors)
PER_CAMERA = SG.value("MIRLT_CAMERA_VERDICT") == "per_camera"   # (e21) the verdict per camera ("whole": e19's one)
CAMERA_TRUST = SG.value("MIRLT_CAMERA_TRUST")   # (e20) "learn": each camera's learned trust per job and area decides
                        # (camera_trust.py); "e19": the whole-camera watch's one verdict for every camera
PAIR_STAND_PART = SG.flag("MIRLT_PAIR_STAND_PART")   # (e20) a close pair that both stand beyond CLOSE_CM is parted
AGREE_K_E20 = 3.035     # (e20) the 2-D 99 % radius in sigmas (camera_trust.AGREE_K; statistical)
WHOLE_W_E20 = 1.0       # (e20) a whole-picture verdict counts as one check per robot it names (camera_trust.WHOLE_W)
WHOLE_MEM_S = 12.0      # (e19) while not trusted, a robot's last drive counts this long (three windows). Chosen; DOE: no
WHOLE_MIN_MOT = 4       # (e19) at least this many pictures with a driving robot in the window. Chosen; DOE: yes (with
                        # the window, below)
WHOLE_WIN_S = 4.0       # (e19) the window: this long of pictures (at least WHOLE_MIN_PICS). CHOSEN (a few robot-steps
WHOLE_MIN_PICS = 8      # at the measured 1.6 pictures a second); DOE: yes, with the hard simulator's wrong-name and
                        # bumped-camera faults against false alarms on the real logs (replay-all)
ELIM_PAIR_GUARD = SG.flag("MIRLT_ELIM_PAIR_GUARD")    # robots in a close pass still count as unaccounted, and a spot
                         # on another robot's OWN pose is never given to an unaccounted robot by elimination
OWN_ON_SPOT_CM = 6.0     # ... "on its own pose": within the firmware's fix gate (4 sigmas, ~6 cm: pose.h), where that
                         # robot could simply be standing. Not chosen anew
OB_K = 4.0               # a candidate "moved" when its far-end mean is 4 of its own camera sigmas from where it stood (as
                         # freeroam_core.MOVED_IN_K: ~3e-5 by chance on Gaussian noise). Statistical; DOE: no
OB_N = 3                 # pictures averaged at the far end (as STEADY_N). Not chosen anew
OB_SD_FLOOR = 0.3        # cm: the smallest camera sigma believed (a spot standing perfectly still for a second is
                         # rounding, not a sigma). CHOSEN; DOE: no
OB_FAR_SHARE = 0.75      # the OB_N farthest sightings average about this share of the drive (pictures on the way out and
                         # back included; the e18 batch: 1.5 of 2.0 cm at 5 pictures a second). Measured in the rig;
                         # DOE: yes, on real out-and-backs (the lights spot's far-end mean against the drive)
OB_SOUND = SG.flag("MIRLT_OUTBACK_SOUND")   # (e19, audit H3) the out-and-back as long as the camera's noise needs
                         # (refused where there is no room), "moved" judged on medians at the time its own track was out
OB_MAX_SOUND_CM = 12.0   # (e19) the longest identity drive: a body's radius (HALF ~6.7 cm) and half again. Chosen; DOE:
                         # no (a safety bound; the room check decides first)
OB_MIN_CM = 2.0          # Nick 12 Nov: 2-4 cm. The drive is OB_K x its noise x sqrt(2 / OB_N) / OB_FAR_SHARE, inside this
OB_MAX_CM = 4.0
OB_V_MIN = 2.0           # cm/s: the firmware's slowest drive (driveLine's floor). Not chosen
OB_V_MAX = 5.0           # cm/s: "slow": the slowest menu speed (freeroam_core.BACK_V). Not chosen anew
OB_SLACK_S = 4.0         # CHOSEN: the watch runs 2 L / v + this (the command's latency, fixes up to ~1.5 s late, the two
                         # steps' starts); it ends earlier once the moved sighting is back. DOE: no (a time-out on a watch)
ML_ON_DEMAND = os.environ.get("MIRLT_ML_ON_DEMAND", "0") == "1"  # (4) OFF: no detector model exists yet
STEADY_S = 1.0           # CHOSEN (Nick's "~1 s"): a sighting standing this long ...
STEADY_N = 3             # ... seen at least this often (the home bridge handled 1-2 pictures a robot a second at 99 % CPU) ...
STEADY_K = 3.0           # ... every sighting within 3 of its source's sigmas of their mean ("within its own noise"). DOE: yes,
                         # with STEADY_S, on bridge/track.csv replays (time to name a held robot vs wrong eliminations)
SPOT_FORGET_S = 1.5      # CHOSEN: a spot not seen this long is forgotten (or 4 x the layer's measured picture gap, longer)
UNACC_MIN_S = 1.0        # CHOSEN: a robot with no named sighting this long (or 3 x its own measured fix gap) is unaccounted
STAND_CM = 1.0           # its own wheels moved under this over the spot's steady window: standing (the camera's own noise)
ELIM_VERIFY_CM = 5.0     # as freeroam_core.VERIFY_CM: once its own wheels drive this far, the camera spot must have followed
ELIM_FOLLOW = 0.5        # CHOSEN: ... at least half as far, else the elimination is revoked (that spot was not it)
WIG_SD_ABS = 0.6         # CHOSEN: a candidate spot's scatter during the wiggle over this (cm) ...
WIG_SD_K = 3.0           # ... and 3 x its own standing scatter = it turned (a lights spot off the middle swings round);
WIG_TH_DEG = 15.0        # CHOSEN: or its heading changed this much; or the source flagged a spin. DOE: yes, on real wiggles
ELIM_RETRY_MAX_S = 60.0  # an elimination wiggle that showed nothing is asked again after WIGGLE_RETRY_S x tries, at most this
REC_N = 20               # (c) the last 20 corrections a robot (chosen: about a quarter hour of the home runs' re-seeds) ...
REC_S = 900.0            # ... within the last 15 min
REC_MIN = 5              # CHOSEN: under 5 corrections, the e09 numbers are used as they are (starting guess)
REC_K = 1.5              # CHOSEN: the gate's ceiling is 1.5 x its 90th percentile correction + 3 sigmas. DOE: yes
BOARD_CAP = 0.5          # the ceiling never passes half the board's shorter side (another robot is always nearer)
ML_ROBOT_HZ = 2.0        # (4) CHOSEN: the detector at most twice a second a robot ...
ML_TOTAL_HZ = 4.0        # ... and 4 a second in all (the home Pi's cores 2-3 were full). DOE: yes, once a model exists
SOURCE_RANK = {"tags": 0, "lock": 1, "motion": 2, "lights": 3, "ml": 4}   # several for one robot: the best one


def sources_for(tracking, old=False, ml=None):
    s = list((OLD_SOURCES if old else SOURCES_BY_TRACKING).get(tracking, ()))
    if (ML_DETECTOR if ml is None else ml) and "ml" not in s and not old:
        s.append("ml")
    return tuple(s)


class Det:
    """One sighting. name: set by tags (the id) and by a lock (the robot it was seeded on); None otherwise."""
    __slots__ = ("src", "t", "x", "y", "th", "name", "sigma", "score", "area", "spin", "lock_id", "cam", "light",
                 "members")

    def __init__(self, src, t, x, y, th=None, name=None, sigma=None, score=1.0, area=1.0, spin=False, lock_id=None,
                 cam=None, light=None):
        self.src, self.t, self.x, self.y, self.th, self.name = src, t, float(x), float(y), th, name
        self.sigma = SOURCE_SIGMA.get(src, 1.0) if sigma is None else float(sigma)
        self.score, self.area, self.spin, self.lock_id = score, area, spin, lock_id
        self.cam, self.light = cam, light       # (e20) the camera that saw it (None: not known, as before e20)
        self.members = None                     # (e20) a cluster's sightings [Det] (several cameras of one object)


class Track:
    def __init__(self, name):
        self.name = name
        self.sure = None            # (t, x, y) last sure sighting
        self.own = []               # [(t, x, y, th, still)] own pose / filter (prediction and identity by motion)
        self.cam = []               # [(t, x, y)] the sightings given to it
        self.src_n = {}
        self.unsure = None          # why no fix now (close pass, contested, wiggle) or None
        self.parked_at = None       # (t0, x, y) camera spot when its wheels stopped
        self.lock_seeded = None     # t of the lock's last seed
        self.seen_t = None
        self.elim = None            # (e17) dict(t, x, y, own0, n0) while an elimination is on probation
        self.rec = []               # (e17, c) [(t, d_cm, travel_cm)] its measured own-vs-camera corrections
        self.fix_ts = []            # (e17) the times of its last named sightings (its own fix gap)


class TrackingLayer:
    def __init__(self, names, sources=("motion", "lock", "lights"), identity=None, parked_watch=True,
                 brick_watch=True, log=None, lock_seed=None, wiggle=None, cluster=True, camera_watch=None,
                 eliminate=None, gate_record=None, board=None, ml_fn=None, ml_on_demand=None, outback=None,
                 pair_guard=None, book=None):
        """log(kind, row); lock_seed(name, t, x, y): ask the lock-on to lock on that robot there (bridge / simulator);
        wiggle(name, why): ask the controller for an identity wiggle of that robot (None: no wiggle can be made, the
        robots keep driving on their filters until motion tells them apart)."""
        self.names = list(names)
        self.sources = tuple(sources)
        self.identity = identity or IDENTITY
        self.parked_watch = parked_watch
        self.brick_watch = brick_watch
        self.log = log or (lambda kind, row: None)
        self.lock_seed_fn = lock_seed
        self.wiggle_fn = wiggle
        self.cluster = cluster
        self.own_resets_on = OWN_RESETS and cluster      # (e10) with the e09 layer only (the old way: as before)
        self.camera_watch = CAMERA_WATCH if camera_watch is None else bool(camera_watch)
        self.cam_ok = True           # the camera watch: False while the picture moves unlike the robots' own wheels
        self.cam_good = 0            # agreeing votes so far: the camera is PROVEN after CAM_GOOD_N (until then a camera
                                     # spot may not move a robot's own pose by itself: controller.camera_proven)
        self.cam_checks = []         # recent checks: True (agrees) / False
        self.cam_ref = {}            # name -> (t, cam x, y, own x, y) where the next check is measured from
        self.T = {n: Track(n) for n in self.names}
        self.pairs = {}             # frozenset(a, b) -> dict(t0, parted_t, state)
        self.wiggles = {}           # name -> dict(t0, pair, at)
        self.unclaimed = []         # [dict(t0, t, x, y, n)] for the brick watch
        self.obstacles = []
        self.counts = dict(frames=0, fixes=0, contested=0, merged=0, close_passes=0, kept=0, swapped=0, wiggles=0,
                           wiggle_kept=0, wiggle_swapped=0, wiggle_none=0, parked_moved=0, bricks=0, seeds=0,
                           tag_jumps=0, refused_close=0, lock_used=0, lock_checked=0)
        self.by_src = {}
        self.last_fix = {}          # name -> (t, src)
        # (e17) observe, then trust
        self.eliminate = (ELIMINATE if eliminate is None else bool(eliminate)) and self.identity != "predict"
        self.gate_record = GATE_RECORD if gate_record is None else bool(gate_record)
        self.board = tuple(board) if board else None
        self.spots = []             # unclaimed sightings followed over time: dict(pts=[(t, x, y, sig, spin, th)], t0, t)
        self.not_robot = []         # spots an elimination was revoked on: dict(x, y, t) (they stayed when it drove)
        self.whole = []             # (e19) (t, sightings, outside every gate) per picture: the whole-camera watch
        self.cam_distrust = None    # (e19) dict(t, why) while the camera as a whole is not trusted
        self.cam_bad = set()        # (e21) the cameras the per-camera verdict refuses now (None: an unnamed camera)
        self.slips = {}             # (e21) {robot: [(t0, t1)]}: its moves that slipped (the run's verdict)
        self.focus = {}             # (e21) {robot: (x, y, reach)}: a searched robot's error shape (its gate, the detector)
        self.mot_last = {}          # (e19) robot -> (t, its last drive's motion agreed with a sighting)
        self.cam_test_want = []     # (e19) robots asked for a short test drive while the camera is not trusted
        self.say_fn = None          # (e19) the runner's say(): the whole-camera verdict on screen
        self.flags = {}             # name -> why the run counts it unaccounted (lost / unsure / held), from the run
        self.elim_wig = None        # dict(robot, cands, t0, tries, why) while an elimination wiggle runs
        self.elim_wait = {}         # robot -> time the next elimination wiggle may be asked
        self.revoked = []           # [dict(robot, t, x, y)] eliminations taken back (take_revoked)
        self.wiggle_spots = None    # while an elimination wiggle is asked: the candidate spots (the run checks each)
        self.wiggle_req = None      # (e18) while one is asked: dict(kind "wiggle" | "outback", spots, L, v)
        self.outback_on = OUTBACK_ID if outback is None else bool(outback)
        self.pair_guard = ELIM_PAIR_GUARD if pair_guard is None else bool(pair_guard)
        self.id_fails = {}          # (e18) robot -> dict(wiggle=n, outback=n): tries that showed nothing
        self.frame_ts = []
        self.ml_fn = ml_fn          # (4) fn(name, t, why, (x, y, r)) -> [Det] or None: the detector, run on demand
        self.ml_on = (ML_ON_DEMAND if ml_on_demand is None else bool(ml_on_demand)) and ml_fn is not None
        self.ml_last = {}
        self.ml_calls = []
        self.want_ml = {}           # name -> why (a heading check, a disagreement), from the run
        # (e20) learned camera trust: the book (camera_trust.TrustBook) is the run's, set here or later (layer.book)
        self.book = book if CAMERA_TRUST != "e19" else None
        self.cam_ref = {}           # (e20) (robot, cam) -> (t, x, y) its last checked sighting by that camera
        self.own_sd_fn = None       # (e20) fn(robot, d_cm) -> its own pose's sigma after d cm (the odometry record)
        self.cam_sigma_fn = None    # (e20) fn(cam, method) -> that camera's measured sigma (cm) or None
        self.member_fn = None       # (e20) fn(robot, Det, t): every camera's sighting of a named robot (heading from motion)
        self.last_cam = {}          # (e20) robot -> camera of its last fix
        self.trust_refused = {}     # (e20) camera -> sightings refused (distrusted for position there)
        self.trust_refused_now = False
        for k in ("eliminated", "elim_wiggles", "elim_wiggle_named", "elim_wiggle_none", "elim_revoked",
                  "elim_confirmed", "elim_ambiguous", "ml_calls", "ml_dets", "outbacks", "outback_named",
                  "outback_none", "outback_refused", "wiggle_refused"):
            self.counts[k] = 0

    # -- inputs from the robots --------------------------------------------------------------------------------------
    def own(self, name, t, x, y, th, still):
        tr = self.T.get(name)
        if tr is None:
            return
        x, y = float(x), float(y)
        if tr.own and self.own_resets_on:
            # (e10) an own pose that moves faster than any robot drives is a RE-SET of its frame (the controller's
            # fpose to the camera), not motion: the history is shifted to stay continuous, so the prediction (last
            # sure spot + own motion since) does not jump. 7 Nov home run, 12:33:47: Left's own pose was set 14 cm
            # to the camera's, read as a 14 cm drive backwards, and its sightings (the side camera's too) fell outside
            # its gate for 13 s: the run's only full stop
            p = tr.own[-1]
            dt = max(0.0, t - p[0])
            dx, dy = x - p[1], y - p[2]
            if OWN_RESET_RULE == "velocity":
                # (e17) where its own recent velocity would carry it: a re-seed while driving is a step off that line
                vx = vy = 0.0
                if len(tr.own) >= 2 and p[0] > tr.own[-2][0]:
                    q = tr.own[-2]
                    vx, vy = (p[1] - q[1]) / (p[0] - q[0]), (p[2] - q[2]) / (p[0] - q[0])
                    if math.hypot(vx, vy) > OWN_MAX_CMS:      # the previous step was itself a re-set
                        vx = vy = 0.0
                off = math.hypot(dx - vx * dt, dy - vy * dt)
                jump = off > OWN_JUMP_CM + OWN_ACCEL * dt * dt + 0.5 * math.hypot(vx, vy) * dt or \
                    math.hypot(dx, dy) > OWN_JUMP_CM + OWN_MAX_CMS * dt
            else:
                jump = math.hypot(dx, dy) > OWN_JUMP_CM + OWN_MAX_CMS * dt
            if jump:
                tr.own = [(a, b + dx, c + dy, d, e) for a, b, c, d, e in tr.own]
                if tr.elim is not None and tr.elim.get("own0") is not None:     # (e17) the probation's start too
                    tr.elim["own0"] = (tr.elim["own0"][0] + dx, tr.elim["own0"][1] + dy)
                self.counts["own_resets"] = self.counts.get("own_resets", 0) + 1
                if tr.sure is not None:     # (e17, c) a re-set is a measured correction: how far its own pose had gone
                    self.record(name, t, math.hypot(dx, dy), self._travel(tr, tr.sure[0], t))
                self.log("tracking", dict(t=round(t, 2), robot=name, what="own pose re-set",
                                          d_cm=round(math.hypot(dx, dy), 1)))
        tr.own.append((t, x, y, th, bool(still)))
        if len(tr.own) > 400:
            del tr.own[:100]
        if not still:
            tr.parked_at = None

    def _own_at(self, tr, t):
        h = tr.own
        if not h:
            return None
        if t <= h[0][0]:
            return h[0]
        if t >= h[-1][0]:
            return h[-1]
        lo, hi = 0, len(h) - 1
        while hi - lo > 1:
            m = (lo + hi) // 2
            if h[m][0] <= t:
                lo = m
            else:
                hi = m
        a, b = h[lo], h[hi]
        f = (t - a[0]) / max(1e-6, b[0] - a[0])
        return (t, a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), b[3], b[4])

    def predict(self, name, t):
        """(x, y, gate) where the robot should be at t: the last sure sighting moved on by its own motion since."""
        tr = self.T[name]
        if tr.sure is None:
            o = self._own_at(tr, t)
            return None if o is None else (o[1], o[2], self.gate_max(name) if self.gate_record else GATE_MAX_CM)
        ts, xs, ys = tr.sure
        a, b = self._own_at(tr, ts), self._own_at(tr, t)
        x, y = (xs + b[1] - a[1], ys + b[2] - a[2]) if (a is not None and b is not None) else (xs, ys)
        if self.identity == "predict" or a is None or b is None:
            gate = min(GATE_MAX_CM, GATE_CM + GATE_GROW_CMS * max(0.0, t - ts))      # e02_multitrack's, by time
        else:
            # (e09) the gate grows with what the robot's own wheels say it did since, not with the clock: a robot
            # standing still keeps a tight gate (a wrong spot near it is not taken), one driving blind a wide one
            trav = self._travel(tr, ts, t)
            if self.gate_record:
                # (e17, c) grown with what THIS robot's own pose is measured to drift (cm per cm driven), not only
                # the chosen 0.15; an unaccounted robot's gate also covers its usual correction; the ceiling from
                # its record and the board (the e09 25 cm is the starting guess, never narrowed)
                per, p90 = self.drift(name)
                unacc = self._unaccounted(name, t)
                root = GATE_ROOT_CM * math.sqrt(max(0.0, t - ts))
                if unacc and trav < STAND_CM and sum(1 for v in self.flags.values() if v) >= 2:
                    # (e18, found by the identity batch) with two or more robots flagged unaccounted, one whose own
                    # wheels have not driven since it was last sure cannot have drifted with time: no time growth
                    # (after ~40 s it reached 20 cm and took the OTHER unaccounted robot's sighting: a swap). Being
                    # pushed is the elimination's / identity moves' job. With fewer (a close pass, e09's test) the e17
                    # growth stays: it is how a standing robot beside another gets its sighting back to settle the pass
                    root = 0.0
                gate = GATE_CM + max(GATE_PER_CM, per) * trav + root
                if unacc and sum(1 for v in self.flags.values() if v) < 2:
                    # (e17) widened to its usual correction; (e18, found by the identity batch) only while it is the one
                    # robot the run cannot account for: with two, the widened gate of one took the OTHER's sighting (its
                    # own pose happened to lie 7 cm from it) and the elimination named the other on the leftover: a swap.
                    # With two or more, who is who is the identity moves' job (wiggle, out-and-back)
                    gate = max(gate, GATE_CM + p90)
                gate = min(self.gate_max(name), gate)
            else:
                gate = min(GATE_MAX_CM, GATE_CM + GATE_PER_CM * trav + GATE_ROOT_CM * math.sqrt(max(0.0, t - ts)))
        return (x, y, gate)

    # -- (e17, c) each robot's measured record -------------------------------------------------------------------
    def record(self, name, t, d_cm, travel_cm=None):
        """A measured correction of `name`'s own pose against the camera (an own-pose re-set, an elimination, a re-seed
        the run made): d_cm, after travel_cm driven since the camera last had it."""
        tr = self.T.get(name)
        if tr is None:
            return
        tr.rec.append((t, float(d_cm), None if travel_cm is None else float(travel_cm)))
        del tr.rec[:-REC_N]

    def drift(self, name, t=None):
        """(cm of own-pose error per cm driven, 90th percentile correction cm) from the record; the e09 numbers
        (0.15 cm per cm; 0 cm) when under REC_MIN corrections."""
        tr = self.T[name]
        now = t if t is not None else (tr.rec[-1][0] if tr.rec else 0.0)
        rec = [q for q in tr.rec if now - q[0] <= REC_S]
        if len(rec) < REC_MIN:
            return GATE_PER_CM, 0.0
        ds = sorted(q[1] for q in rec)
        p90 = ds[min(len(ds) - 1, int(0.9 * len(ds)))]
        rates = sorted(q[1] / q[2] for q in rec if q[2] is not None and q[2] >= ELIM_VERIFY_CM)
        per = rates[min(len(rates) - 1, int(0.9 * len(rates)))] if len(rates) >= REC_MIN else GATE_PER_CM
        return min(1.0, per), p90

    def gate_max(self, name):
        f = self.__dict__.get("focus", {}).get(name)
        if f is not None:
            return max(self._gate_max(name), float(f[2]))      # (e21) searched: its whole error shape
        return self._gate_max(name)

    def _gate_max(self, name):
        cap = BOARD_CAP * min(self.board[2] - self.board[0], self.board[3] - self.board[1]) if self.board else 61.0
        if not self.gate_record:
            return GATE_MAX_CM
        _, p90 = self.drift(name)
        sig = max(SOURCE_SIGMA.values())
        return min(cap, max(GATE_MAX_CM, REC_K * p90 + 3.0 * sig))

    def _travel(self, tr, ta, tb):
        d, prev = 0.0, None
        for q in tr.own:
            if q[0] < ta:
                prev = q
                continue
            if q[0] > tb:
                break
            if prev is not None:
                d += math.hypot(q[1] - prev[1], q[2] - prev[2])
            prev = q
        return d

    def seed(self, name, t, x, y, why="tag"):
        """A sure, named sighting (a tag read, a lights find, the start spot): the robot's track starts there and the
        lock-on is seeded on it (when the robot is not close to another: a seed must not land on a look-alike)."""
        tr = self.T.get(name)
        if tr is None:
            return
        tr.sure = (t, float(x), float(y))
        if "lock" in self.sources and self.lock_seed_fn is not None and not self._close_to_other(name, x, y) and (
                tr.lock_seeded is None or t - tr.lock_seeded > 1.0):
            tr.lock_seeded = t
            self.counts["seeds"] += 1
            self.lock_seed_fn(name, t, x, y)

    def _close_to_other(self, name, x, y, d=CLOSE_CM):
        for n, o in self.T.items():
            if n != name and o.sure is not None and math.hypot(o.sure[1] - x, o.sure[2] - y) < d:
                return True
        return False

    # -- one picture's sightings ---------------------------------------------------------------------------------------
    def feed(self, t, dets):
        """All sightings of one moment (any sources). Returns [dict(robot, t, x, y, th, sigma, src)]."""
        self.counts["frames"] += 1
        dets = [d for d in dets if d.src in self.sources]
        self.frame_ts.append(t)
        del self.frame_ts[:-40]
        preds = {n: self.predict(n, t) for n in self.names}
        if self.ml_on:
            dets = dets + self._ml_on_demand(t, preds)        # (e17, 4) the detector, only where it is needed
        all_dets, refused = dets, []
        book = self._percam(dets)
        if book is not None:
            # (e20) a camera whose own evidence says it is wrong for POSITION where this sighting is: refused here (it is
            # still checked below, so it can earn trust back); unproven and trusted cameras are used as before
            keep = []
            for d in dets:
                if d.cam is not None and d.src != "tags" and book.refused(d.cam, d.src, "position", d.x, d.y, d.light):
                    refused.append(d)
                    self.trust_refused[d.cam] = self.trust_refused.get(d.cam, 0) + 1
                else:
                    keep.append(d)
            dets = keep
        self._pairs(t, preds)
        out, named = {}, set()
        # 1. tags: the id is the name (a read far from the prediction is still believed: the tag knows; said)
        for d in dets:
            if d.src == "tags" and d.name in self.T:
                p = preds.get(d.name)
                if p is not None and math.hypot(d.x - p[0], d.y - p[1]) > GATE_MAX_CM:
                    self.counts["tag_jumps"] += 1
                    self.log("tracking", dict(t=round(t, 2), robot=d.name, what="tag read away from the prediction",
                                              d_cm=round(math.hypot(d.x - p[0], d.y - p[1]), 1)))
                out[d.name] = d
                named.add(d.name)
        # 2. the locks: a lock's name stands unless its robot is in a close pass or a wiggle (then it is checked as an
        #    unnamed sighting) or it is off its prediction's gate
        free = []
        for d in dets:
            if d.src == "tags":
                continue
            if d.src == "lock" and d.name in self.T and d.name not in named:
                p = preds.get(d.name)
                tr = self.T[d.name]
                if tr.unsure is None and p is not None and math.hypot(d.x - p[0], d.y - p[1]) <= p[2] and \
                        not self._in_pair(d.name):
                    out[d.name] = d
                    named.add(d.name)
                    self.counts["lock_used"] += 1
                    continue
                self.counts["lock_checked"] += 1
            free.append(d)
        # 3. unnamed (and checked locks): matched to predictions (several sources' sightings of one object: one)
        if self.cluster:
            free = self._clusters(free)
        if PER_CAMERA and self.cam_bad:
            # (e21) a camera the verdict implicates is left out before assignment: another camera's sighting is used
            free = [d for d in free if d.cam not in self.cam_bad]
        want = [n for n in self.names if n not in named and preds.get(n) is not None]
        claimed = set()
        if want and free:
            asg = self._assign(t, want, preds, free)
            for n, d in asg.items():
                out[n] = d
                claimed.add(id(d))
        # 3a. (e19, audit 10) the whole camera: most sightings outside every robot's gate for a window = a wrong camera
        #     (e20: its verdict also counts as evidence against each camera in the picture)
        if book is not None:
            self._trust_checks(t, out, preds, refused, free, book, all_dets)
        if WHOLE_CAMERA_WATCH and PER_CAMERA:
            bad = self._whole_watch_cams(t, all_dets if book is not None else dets, preds, claimed, free)
            if bad:
                for n in [n for n, d in out.items() if d.src != "tags" and (None in bad or d.cam in bad)]:
                    out.pop(n)
                self.counts["camera_refused_alone"] = self.counts.get("camera_refused_alone", 0) + 1
        elif WHOLE_CAMERA_WATCH and self._whole_watch(t, all_dets if book is not None else dets, preds, claimed, free):
            for n in [n for n, d in out.items() if d.src != "tags"]:
                out.pop(n)
            self.counts["whole_camera_refused"] = self.counts.get("whole_camera_refused", 0) + 1
        how = {}
        if self.eliminate and (self.cam_distrust is None or (PER_CAMERA and None not in self.cam_bad)):
            rest = [d for d in free if id(d) not in claimed and d.src != "tags" and d.cam not in self.cam_bad]
            for n, (d, hw) in self._eliminate(t, preds, rest, set(out)).items():
                out[n] = d
                claimed.add(id(d))
                how[n] = hw
        # 4. identity guard: robots in an unresolved close pass or wiggle take no camera fix
        fixes = []
        for n, d in out.items():
            tr = self.T[n]
            tr.fix_ts.append(t)
            del tr.fix_ts[:-40]
            tr.sure = (t, d.x, d.y)          # the track goes on (who it is is checked when the pair parts) ...
            tr.seen_t = t
            tr.cam.append((t, d.x, d.y))
            if len(tr.cam) > 200:
                del tr.cam[:50]
            if (tr.unsure is not None or self._in_pair(n)) and d.src != "tags":
                self.counts["refused_close"] += 1    # ... but no fix: its filter drives on its own sensors meanwhile,
                continue                             # so its own motion stays clean evidence for the check
            if self.camera_watch and not self._camera_check(n, t, d) and d.src != "tags" and n not in how:
                self.counts["refused_camera"] = self.counts.get("refused_camera", 0) + 1
                continue
            tr.src_n[d.src] = tr.src_n.get(d.src, 0) + 1
            self.by_src[d.src] = self.by_src.get(d.src, 0) + 1
            if d.src == "tags":
                tr.unsure = None
                self.seed(n, t, d.x, d.y, "tag")
            elif d.src == "motion" and "lock" in self.sources and (tr.lock_seeded is None or t - tr.lock_seeded > 3.0):
                # the detector sees it sure and nobody near: (re)lock on to it
                if not self._close_to_other(n, d.x, d.y):
                    self.seed(n, t, d.x, d.y, "motion")
            self.last_fix[n] = (t, d.src)
            self.last_cam[n] = d.cam
            self._parked(n, t, d)
            f = dict(robot=n, t=t, x=d.x, y=d.y, th=d.th, sigma=d.sigma, src=d.src)
            if d.cam is not None:
                f["cam"] = d.cam                 # (e20) which camera: the run's checks weigh it by that camera's trust
                f["sigma"] = self._cam_sigma(d)  # and its MEASURED noise (never below its method's)
                if d.light is not None:
                    f["light"] = d.light
            if n in how:
                f["how"] = how[n]            # (e17) the run takes it as a correction (its pose set, never "moved in")
                f["d_cm"] = round(tr.rec[-1][1], 1) if tr.rec and tr.rec[-1][0] == t else None
            fixes.append(f)
        self.counts["fixes"] += len(fixes)
        # 5. wiggles in progress, close passes that parted, things nobody claims
        self._wiggle_watch(t, dets)
        self._parted(t)
        if self.eliminate:
            self._probation(t)
        if self.brick_watch:
            self._bricks(t, [d for d in free if id(d) not in claimed and d.src in ("motion", "ml")], preds)
        return fixes

    def _whole_watch_cams(self, t, dets, preds, claimed, free):
        """(e21, e20's open problem: the camera-swap scenario stood 526 s because one verdict refused the good camera too)
        e19's whole-camera watch run for EACH camera's own sightings, with its own window and memory: the verdict refuses
        only the camera(s) whose own sightings moved unlike the robots, and each is trusted again by its own agreeing
        motion (or every robot within its doubt). Sightings with no camera name are one camera (None), as before.
        Returns the set of cameras refused now."""
        st = self.__dict__.setdefault("cam_watch", {})
        cams = sorted({d.cam for d in dets if d.src != "ml"}, key=lambda c: str(c))
        keep = ("whole", "whole_pics", "mot_last", "cam_distrust", "cam_test_want")
        mine = {k: self.__dict__.get(k) for k in keep}
        tests = set()
        for cam in cams:
            c = st.setdefault(cam, dict(whole=[], whole_pics=[], mot_last={}, cam_distrust=None, cam_test_want=[]))
            for k in keep:
                self.__dict__[k] = c[k]
            self._watch_cam = cam
            try:
                self._whole_watch(t, [d for d in dets if d.cam == cam], preds, claimed, free)
            finally:
                for k in keep:
                    c[k] = self.__dict__.get(k)
                self._watch_cam = None
        for k in keep:
            self.__dict__[k] = mine[k]
        bad = {cam for cam, c in st.items() if c["cam_distrust"] is not None}
        for cam in bad:
            tests |= set(st[cam]["cam_test_want"] or [])
        if bad and self.cam_distrust is None:
            first = min(st[c]["cam_distrust"]["t"] for c in bad)
            self.cam_distrust = dict(t=first, why="; ".join("camera %s: %s" % (c, st[c]["cam_distrust"]["why"])
                                                            for c in sorted(bad, key=str)), cams=sorted(bad, key=str))
        elif bad:
            self.cam_distrust["cams"] = sorted(bad, key=str)
        elif self.cam_distrust is not None:
            self.cam_distrust = None
        self.cam_bad = bad
        self.cam_test_want = sorted(tests) if bad else []
        return bad

    def set_slips(self, name, windows):
        """(e21) The run's slip windows for a robot (picture times): no evidence against a camera inside them."""
        self.slips[name] = list(windows or ())

    def set_focus(self, name, f):
        """(e21) The run's search for a robot (its error shape's region) or None."""
        if f:
            self.focus[name] = f
        else:
            self.focus.pop(name, None)

    def _whole_watch(self, t, dets, preds, claimed, free):
        """(e19, audit finding 10: "the camera watch votes only on sightings already named to a robot; mirrored
        sightings fall outside every gate, so there are no votes") Per picture: how many sightings fall outside EVERY
        robot's gate (its prediction's gate, at least CLOSE_CM). Over the last WHOLE_WIN_S of pictures (at least
        WHOLE_MIN_PICS), with two or more robots predicted: when most sightings disagree in every one of them, the camera
        as a whole is not trusted (no fixes from it, no eliminations; the robots drive on their own sensors under their
        doubt), said on screen and in the log; trusted again once most sightings agree again over a window. Returns True
        while it is not trusted."""
        # Evidence, per picture: (a) MOTION: each robot that drove WHOLE_MOVE_CM or more over the last half window by its
        # own wheels and gyro, and whether a sighting moved as it did (mirrored or turned pictures move the wrong way;
        # a robot whose own pose alone is off still moves the right way, so it is not a camera fault); (b) POSITION:
        # most robots find a sighting within their own stated doubt (proof_fn: the run's 95 % doubt and the camera's
        # noise; else the layer's gate). The camera is not trusted when, over the window, two or more robots drove and
        # NONE of them had a sighting that moved with it while the camera saw things move; trusted again when two or
        # more robots' motion, or most robots' positions in every picture, agree again.
        fn = getattr(self, "proof_fn", None)
        sights = [d for d in dets if d.src != "ml"]
        live = []
        for n, p in preds.items():
            o = self.T[n].own[-1] if self.T[n].own else None
            q = None
            if o is not None and fn is not None:
                try:
                    q = (o[1], o[2], max(CLOSE_CM, float(fn(n, 1.0))))
                except Exception:  # noqa: BLE001
                    q = None
            if q is None and p is not None:
                q = (p[0], p[1], max(p[2], CLOSE_CM))
            if q is not None:
                live.append(q)
        pos_good = None
        if len(live) >= 2:
            miss = sum(1 for q in live if not any(math.hypot(d.x - q[0], d.y - q[1]) <= q[2] for d in sights))
            pos_good = miss == 0
        self.whole_pics = [q for q in getattr(self, "whole_pics", []) if t - q[0] <= WHOLE_WIN_S] + \
            [(t, [(d.x, d.y) for d in sights])]
        U, M, seen_move = set(), set(), False
        old = [q for q in self.whole_pics if q[0] <= t - 0.5 * WHOLE_WIN_S]
        if old and sights:
            t0, s0 = old[-1]
            for n in self.names:
                a, b = self._own_at(self.T[n], t0), self._own_at(self.T[n], t)
                if a is None or b is None or a[0] > t0 + 0.5:
                    continue
                D = (b[1] - a[1], b[2] - a[2])
                if math.hypot(*D) < WHOLE_MOVE_CM:
                    continue
                LD = math.hypot(*D)
                tol = max(2.0, 0.35 * LD)
                hit, best = False, None
                for d in sights:
                    near = min(s0, key=lambda q: math.hypot(q[0] - d.x, q[1] - d.y)) if s0 else None
                    if near is None or math.hypot(near[0] - d.x, near[1] - d.y) > LD + 5.0:
                        continue
                    mv = (d.x - near[0], d.y - near[1])
                    seen_move = seen_move or math.hypot(*mv) >= WHOLE_MOVE_CM
                    if math.hypot(mv[0] - D[0], mv[1] - D[1]) <= tol:
                        hit = True
                        if self._percam(sights) is not None and d.cam is not None:
                            # (e20) a robot that drove and a sighting of this camera that moved as it did: evidence for
                            # the camera that needs no name (how a camera not trusted earns trust back while robots
                            # are lost)
                            self.book.observe(d.cam, d.src, "position", d.x, d.y, True, 1.0, d.light, t,
                                              "a robot's own motion (the whole picture)")
                        break
                    dl = abs(math.hypot(*mv) - LD)
                    if dl <= 0.4 * LD and (best is None or dl < best[0]):
                        best = (dl, mv, d)
                if hit:
                    M.add(n)
                    self.mot_last[n] = (t, True)
                elif best is not None and self.book is not None and any(
                        o != n and self.T[o].own and math.hypot(self._own_at(self.T[o], t)[1] - best[2].x,
                                                                self._own_at(self.T[o], t)[2] - best[2].y) < CLOSE_CM
                        for o in self.names):
                    # (e20, the 11 Nov profile, seed 1: a false whole-camera verdict at 181 s while three robots drove
                    # within 16 cm of each other kept every camera untrusted for 738 s) the wrong-way sighting lies on a
                    # neighbour: it may be the neighbour's own, so it is no evidence against the camera
                    self.counts["whole_neighbour_skipped"] = self.counts.get("whole_neighbour_skipped", 0) + 1
                elif best is not None:
                    self.mot_last[n] = (t, False)
                    # the same length, another direction: how a ONE camera-wide transform would explain it. A mirror
                    # across a line at phi maps direction a to 2 phi - a (phi = mean of the two, mod 180); a turned
                    # (bumped) camera turns every direction by the same theta
                    aD = math.degrees(math.atan2(D[1], D[0]))
                    aS = math.degrees(math.atan2(best[1][1], best[1][0]))
                    U.add((n, ((aD + aS) / 2.0) % 180.0, (aS - aD + 180.0) % 360.0 - 180.0))
        if U or M or pos_good is not None:
            self.whole.append((t, U, M, pos_good, seen_move))
        self.whole = [q for q in self.whole if t - q[0] <= WHOLE_WIN_S][-200:]
        win = self.whole
        uUx = [u for q in win for u in q[1]]
        uU = {u[0] for u in uUx}
        uM = set().union(*[q[2] for q in win]) if win else set()
        n_mot = sum(1 for q in win if q[1] or q[2])
        # every robot that drove (three or more) moved against its sighting, or two did and ONE camera-wide transform
        # explains both
        all_bad = n_mot >= WHOLE_MIN_MOT and len(uU) >= 2 and not uM and any(q[4] for q in win) and \
            (len(uU) >= 3 or self._one_transform(uUx))
        # trusted again on motion (two or more robots' drives seen moving the right way, none the wrong way), or when
        # EVERY robot has a sighting within its own doubt in every picture of the window (robots that stand while the
        # camera is not trusted show no motion; "every", not "most": robots near a mirror's axis agree even then)
        pos_ok = [q[3] for q in win if q[3] is not None]
        all_good = (len(uM) >= 2 and not uU) or (len(pos_ok) >= WHOLE_MIN_PICS and all(pos_ok) and not uU)
        if self.cam_distrust is not None:
            # robots stand while the camera is not trusted (no motion, no evidence): each robot's LAST drive since then,
            # within WHOLE_MEM_S, counts; two or more that agreed and none that did not = trusted again. Without them,
            # the run is asked for short test drives (cam_test_want: the controller's camera_test_step)
            since = [(n, v) for n, v in self.mot_last.items() if v[0] >= self.cam_distrust["t"] and t - v[0] <= WHOLE_MEM_S]
            ok_n = [n for n, v in since if v[1]]
            all_good = all_good or (len(ok_n) >= 2 and len(ok_n) == len(since))
            self.cam_test_want = [n for n in self.names if n not in dict(since)]
        else:
            self.cam_test_want = []

        if all_bad and self._percam(sights) is not None:
            # (e20) every robot that drove moved unlike every sighting: the verdict below stands as in e19 (a wrong name
            # or a bumped camera, no fixes from it), and each camera in the picture also takes it as evidence (as many
            # checks as robots it names), so its learned score follows
            for cm in sorted({(d.cam, d.src) for d in sights if d.cam is not None}):
                ds = [d for d in sights if (d.cam, d.src) == cm]
                self.book.observe(cm[0], cm[1], "position", ds[0].x, ds[0].y, False, WHOLE_W_E20 * len(uU), ds[0].light,
                                  t, "a robot's own motion (the whole picture)")
            self.counts["whole_camera_evidence"] = self.counts.get("whole_camera_evidence", 0) + 1
        if self.cam_distrust is None and all_bad:
            self.cam_distrust = dict(t=t, why="over the last %.1f s, %s drove by their own wheels and gyro and none of "
                                     "them had a sighting that moved as it did, while the camera saw things move" % (
                                         WHOLE_WIN_S, " and ".join(sorted(uU))))
            self.counts["whole_camera_distrust"] = self.counts.get("whole_camera_distrust", 0) + 1
            wc = self.__dict__.get("_watch_cam")
            self.log("tracking", dict(t=round(t, 2), robot="-", what="CAMERA NOT TRUSTED" + (
                " (%s)" % wc if wc is not None else ""), why=self.cam_distrust["why"]))
            if self.say_fn is not None:
                self.say_fn("!! CAMERA %sNOT TRUSTED: %s (a wrong camera name or a bumped camera?). No camera fixes or "
                            "eliminations from it; the robots drive on their own sensors within their doubt until it "
                            "agrees again." % (("%s " % wc) if wc is not None else "", self.cam_distrust["why"]))
        elif self.cam_distrust is not None and all_good:
            self.log("tracking", dict(t=round(t, 2), robot="-", what="camera trusted again",
                                      why="most sightings agree with the robots again over %d pictures" % len(win)))
            if self.say_fn is not None:
                self.say_fn("Camera trusted again: its sightings agree with the robots again (%.0f s after it was "
                            "not trusted)." % (t - self.cam_distrust["t"]))
            self.cam_distrust = None
        return self.cam_distrust is not None

    # -- (e20) learned camera trust: the checks ------------------------------------------------------------------------
    def _percam(self, dets):
        """The trust book when this picture's sightings say which camera saw them (else None: e19's way)."""
        if self.book is None:
            return None
        return self.book if any(getattr(d, "cam", None) is not None for d in dets) else None

    def _cam_sigma(self, d):
        """(e20) The sighting's sigma: its method's measured one, raised (never lowered) to its camera's own measured
        scatter across its line of travel (camera_trust.MotionHeading: a steep camera's depth). The scatter across the
        line misses the timing error ALONG it, so it never replaces the method's sigma."""
        base = float(d.sigma or SOURCE_SIGMA.get(d.src, 1.0))
        fn = self.cam_sigma_fn
        if fn is not None:
            try:
                v = fn(d.cam, d.src)
                if v:
                    return max(base, float(v))
            except Exception:  # noqa: BLE001
                pass
        return base

    def _own_sd(self, n, d_cm):
        fn = self.own_sd_fn
        if fn is not None:
            try:
                return float(fn(n, d_cm))
            except Exception:  # noqa: BLE001
                pass
        return (2.0 + 0.07 * d_cm) / 2.45        # odo_trust's prior (95 % radius) as a sigma

    def _rel_check(self, n, d, t, book, ref_words="its own wheels and gyro"):
        """(e20) The camera's displacement of robot n between two of its sightings against the robot's own displacement
        (its own pose, wheels and gyro) over the same picture times. Checked once the robot moved enough to show it
        (the camera's 2-D 99 % noise over two sightings); a standing robot gives no evidence either way."""
        tr = self.T[n]
        if not tr.own:
            return None
        key = (n, d.cam, d.src)
        ref = self.cam_ref.get(key)
        if ref is None or ref[0] < tr.own[0][0] or t <= ref[0]:
            self.cam_ref[key] = (t, d.x, d.y)
            return None
        a, b = self._own_at(tr, ref[0]), self._own_at(tr, t)
        D = (b[1] - a[1], b[2] - a[2])
        dl = math.hypot(*D)
        sc = self._cam_sigma(d)
        dmin = AGREE_K_E20 * sc * math.sqrt(2.0)
        if dl < dmin:
            if t - ref[0] > WHOLE_MEM_S:
                self.cam_ref[key] = (t, d.x, d.y)
            return None
        so = self._own_sd(n, dl)
        tol = AGREE_K_E20 * math.sqrt(2.0 * sc * sc + so * so)
        err = math.hypot(d.x - ref[1] - D[0], d.y - ref[2] - D[1])
        ok = err <= tol
        if not ok and any(a <= t and ref[0] <= b for a, b in self.slips.get(n, ())):
            # (e21, Nick 12 Nov) the robot's own sensors said it slipped over these pictures: a slip, not the camera
            self.counts["slip_spared"] = self.counts.get("slip_spared", 0) + 1
            self.cam_ref[key] = (t, d.x, d.y)
            return None
        book.observe(d.cam, d.src, "position", d.x, d.y, ok, 1.0, d.light, t, ref_words)
        self.cam_ref[key] = (t, d.x, d.y)
        self.counts["trust_checks"] = self.counts.get("trust_checks", 0) + 1
        return ok

    def _trust_checks(self, t, out, preds, refused, free, book, all_dets=()):
        """(e20) Every camera's sightings of named robots checked against independent evidence: the robot's own motion,
        another camera at the same picture, a roof tag's name. Refused sightings (a camera not trusted there) are
        matched to the nearest prediction for the check only, so a camera can earn trust back."""
        self.trust_refused_now = bool(refused)
        for n, d in out.items():
            mem = d.members or [d]
            for m in mem:
                if m.cam is not None and m.src != "tags":
                    self._rel_check(n, m, t, book)
                    if self.member_fn is not None:
                        try:
                            self.member_fn(n, m, t)      # (e20, 2) heading from motion on EVERY camera that saw it
                        except Exception:  # noqa: BLE001
                            pass
            cams = [m for m in mem if m.cam is not None]
            # every OTHER camera's nearest sighting of this robot inside its gate (whose nearest prediction is this
            # robot), not only those clustered with it: a bumped camera's sighting sits beyond the cluster's 3.5 cm
            p = preds.get(n)
            if p is not None:
                seen = {m.cam for m in cams}
                for cm in {e.cam for e in all_dets if e.cam is not None and e.src != "tags"} - seen:
                    best = None
                    for e in all_dets:
                        if e.cam != cm or e.src == "tags":
                            continue
                        de = math.hypot(e.x - p[0], e.y - p[1])
                        if de > max(p[2], CLOSE_CM):
                            continue
                        if any(m_ != n and q is not None and math.hypot(e.x - q[0], e.y - q[1]) < de
                               for m_, q in preds.items()):
                            continue
                        if best is None or de < best[0]:
                            best = (de, e)
                    if best is not None:
                        cams.append(best[1])
            self._frame_checks(t, cams, book)
        for d in refused:
            best = None
            for n, p in preds.items():
                if p is None:
                    continue
                dd = math.hypot(d.x - p[0], d.y - p[1])
                if dd <= max(p[2], CLOSE_CM) and (best is None or dd < best[0]):
                    best = (dd, n)
            if best is not None:
                self._rel_check(best[1], d, t, book, "its own wheels and gyro (refused sighting)")
        # identity: a roof tag names a robot; another camera's sighting on it must have been named the same
        tags = [d for d in out.values() if d.src == "tags"]
        for tg in tags:
            for e in free:
                for m in (e.members or [e]):
                    if m.cam is None or m.src == "tags" or math.hypot(m.x - tg.x, m.y - tg.y) > CLUSTER_CM:
                        continue
                    who = next((k for k, v in out.items() if v is e), None)
                    if who is not None:
                        book.observe(m.cam, m.src, "identity", m.x, m.y, who == tg.name, 1.0, m.light, t,
                                     "a roof tag's name")

    def _frame_checks(self, t, cams, book):
        """(e20) The cameras that saw one robot in one picture, against each other ("position-frame": where the camera
        puts the board, which relative motion cannot show: a bumped or offset camera). Cameras that agree pairwise
        (2-D 99 % of their two measured noises) form groups; the largest group of two or more agrees; a camera outside
        it that disagrees with every one of them disagrees (a consensus of two or more other cameras against it). Two
        cameras that disagree with nobody else to side with either: no evidence (which one moved cannot be told)."""
        one = {}
        for m in cams:
            one.setdefault(m.cam, m)
        cs = list(one.values())
        if len(cs) < 2:
            return
        ok = {}
        for i in range(len(cs)):
            for j in range(i + 1, len(cs)):
                a_, b_ = cs[i], cs[j]
                tol = AGREE_K_E20 * math.sqrt(self._cam_sigma(a_) ** 2 + self._cam_sigma(b_) ** 2)
                ok[(i, j)] = ok[(j, i)] = math.hypot(a_.x - b_.x, a_.y - b_.y) <= tol
        best = []
        for k in range(len(cs), 1, -1):
            for grp in itertools.combinations(range(len(cs)), k):
                if all(ok[(i, j)] for i, j in itertools.combinations(grp, 2)):
                    best = list(grp)
                    break
            if best:
                break
        if len(best) < 2:
            return
        for i, m in enumerate(cs):
            if i in best:
                book.observe(m.cam, m.src, "position-frame", m.x, m.y, True, 1.0, m.light, t, "other cameras")
            elif all(not ok[(i, j)] for j in best):
                book.observe(m.cam, m.src, "position-frame", m.x, m.y, False, 1.0, m.light, t,
                             "other cameras (%d agree with each other)" % len(best))

    def identity_evidence(self, n, ok, t, why):
        """(e20) A naming of n was confirmed (out-and-back, wiggle, elimination kept) or disproved (swap, revoked):
        evidence for the camera that gave its last fix."""
        cam = self.last_cam.get(n)
        if self.book is not None and cam is not None:
            tr = self.T.get(n)
            x, y = (tr.sure[1], tr.sure[2]) if tr is not None and tr.sure is not None else (None, None)
            self.book.observe(cam, (self.last_fix.get(n) or (None, "motion"))[1], "identity", x, y, ok, 1.0, None, t,
                              why)

    @staticmethod
    def _one_transform(ux):
        """(e19) Two or more robots' wrong-way motions explained by ONE camera-wide transform: a common mirror axis
        (phi, mod 180) or a common turn (theta, past WHOLE_TURN_DEG), each within WHOLE_TRANSFORM_DEG. A robot whose own
        heading is off turns its own motion by its own angle: two of them rarely agree (and never across a window)."""
        by = {}
        for n, phi, th in ux:
            by.setdefault(n, []).append((phi, th))
        if len(by) < 2:
            return False

        def circ_mean(angles, period):
            k = 2.0 * math.pi / period
            c = sum(math.cos(k * a) for a in angles)
            s_ = sum(math.sin(k * a) for a in angles)
            return math.atan2(s_, c) / k % period

        def spread(angles, period, m):
            return max(abs((a - m + period / 2.0) % period - period / 2.0) for a in angles)
        phis = [circ_mean([q[0] for q in v], 180.0) for v in by.values()]
        ths = [circ_mean([q[1] for q in v], 360.0) for v in by.values()]
        mp, mt = circ_mean(phis, 180.0), circ_mean(ths, 360.0)
        mirror = spread(phis, 180.0, mp) <= WHOLE_TRANSFORM_DEG
        turned = spread(ths, 360.0, mt) <= WHOLE_TRANSFORM_DEG and \
            abs((mt + 180.0) % 360.0 - 180.0) >= WHOLE_TURN_DEG
        return mirror or turned

    # -- (e17) observe, then trust: assignment by elimination ----------------------------------------------------
    def set_unaccounted(self, name, why=None):
        """The run says whether it counts `name` unaccounted for (lost, unsure near an edge, held, needs a hand)."""
        if name in self.T:
            if why:
                self.flags[name] = str(why)
            else:
                self.flags.pop(name, None)

    def _fix_gap(self, tr):
        ts = tr.fix_ts[-21:]
        gaps = sorted(b - a for a, b in zip(ts, ts[1:]) if b > a)
        return gaps[len(gaps) // 2] if gaps else None

    def unacc_after(self, name):
        g = self._fix_gap(self.T[name])
        return max(UNACC_MIN_S, 3.0 * g) if g is not None else UNACC_MIN_S

    def _unaccounted(self, name, t):
        if self.flags.get(name):
            return True
        tr = self.T[name]
        last = max(tr.seen_t if tr.seen_t is not None else -1e18, tr.sure[0] if tr.sure is not None else -1e18)
        return t - last > self.unacc_after(name)        # (a seed is a sure sighting too)

    def _standing(self, tr, t0, t1):
        """Its own wheels moved under STAND_CM between t0 and t1 (and its own records reach that far)."""
        if not tr.own or tr.own[-1][0] < t1 - 1.0:
            return False
        a, b = self._own_at(tr, t0), self._own_at(tr, t1)
        if a is None or b is None:
            return False
        return self._travel(tr, t0, t1) < STAND_CM and math.hypot(b[1] - a[1], b[2] - a[2]) < STAND_CM

    def _forget_s(self):
        ts = self.frame_ts[-21:]
        gaps = sorted(b - a for a, b in zip(ts, ts[1:]) if b > a)
        return max(SPOT_FORGET_S, 4.0 * gaps[len(gaps) // 2]) if gaps else SPOT_FORGET_S

    def _spots_update(self, t, rest):
        fs = self._forget_s()
        self.spots = [sp for sp in self.spots if t - sp["t"] <= fs]
        for d in rest:
            sp = None
            for q in self.spots:
                mx, my = q["mx"], q["my"]
                if math.hypot(d.x - mx, d.y - my) <= max(2.0, STEADY_K * max(d.sigma, q["sig"])) and q["t"] < t:
                    sp = q
                    break
            if sp is None:
                sp = dict(pts=[], t0=t, t=t, mx=d.x, my=d.y, sig=d.sigma)
                self.spots.append(sp)
            sp["pts"].append((t, d.x, d.y, d.sigma, bool(d.spin), d.th, d.src))
            sp["pts"] = [q for q in sp["pts"] if t - q[0] <= max(3.0, 2 * STEADY_S)]
            sp["t"] = t
            sp["last"] = d
            sp["mx"] = sum(q[1] for q in sp["pts"]) / len(sp["pts"])
            sp["my"] = sum(q[2] for q in sp["pts"]) / len(sp["pts"])
            sp["sig"] = max(q[3] for q in sp["pts"])
        for nr in self.not_robot:
            if any(math.hypot(sp["mx"] - nr["x"], sp["my"] - nr["y"]) < 4.0 for sp in self.spots):
                nr["t"] = t
        self.not_robot = [nr for nr in self.not_robot if t - nr["t"] <= 10.0]

    def _steady(self, sp, t):
        """A spot standing STEADY_S, seen STEADY_N times, every sighting within STEADY_K sigmas of their mean."""
        pts = [q for q in sp["pts"] if t - q[0] <= STEADY_S + 0.5]
        if len(pts) < STEADY_N or pts[-1][0] - pts[0][0] < STEADY_S:
            return False
        mx = sum(q[1] for q in pts) / len(pts)
        my = sum(q[2] for q in pts) / len(pts)
        return all(math.hypot(q[1] - mx, q[2] - my) <= STEADY_K * q[3] for q in pts)

    @staticmethod
    def _sd(pts):
        if len(pts) < 2:
            return 0.0
        mx = sum(q[1] for q in pts) / len(pts)
        my = sum(q[2] for q in pts) / len(pts)
        return math.sqrt(sum((q[1] - mx) ** 2 + (q[2] - my) ** 2 for q in pts) / len(pts))

    def _eliminate(self, t, preds, rest, named):
        """(e17, a, b) Returns {robot: (Det, how)}. A STEADY spot that no accounted robot's prediction can claim (inside
        its gate or within CLOSE_CM: the anti-swap rule), with exactly one STANDING robot unaccounted for, is that robot
        however far its own pose has drifted. Two or more unaccounted, or two such spots: a wiggle of one of them (the
        spot that turns is it); with no wiggle possible nothing is guessed."""
        self._spots_update(t, rest)
        out = {}
        if self.elim_wig is not None:
            r = self._elim_wiggle_watch(t)
            if r is not None:
                out[r[0]] = (r[1], "wiggle")
            return out
        acc = [n for n in self.names if n not in named and not self._unaccounted(n, t)] + list(named)
        unacc = [n for n in self.names if n not in named and self._unaccounted(n, t) and not self._in_pair(n) and
                 n not in self.wiggles and self.T[n].own]
        if not unacc:
            return out
        # (e18) every robot the run cannot account for counts, a close pass or a wiggle included: they cannot be named
        # here, but they can be the steady spot (5 Nov 14:55: Left and Center in an unresolved pass were left out, so
        # Right looked like the only one and took Left's spot)
        n_unacc = len(unacc) + (sum(1 for n in self.names if n not in named and n not in unacc and
                                     self._unaccounted(n, t) and self.T[n].own) if self.pair_guard else 0)
        cands = []
        for sp in self.spots:
            if not self._steady(sp, t):
                continue
            if any(math.hypot(sp["mx"] - nr["x"], sp["my"] - nr["y"]) < 4.0 for nr in self.not_robot):
                continue
            claim = False
            for m in acc:
                p = preds.get(m)
                if p is not None and math.hypot(sp["mx"] - p[0], sp["my"] - p[1]) <= max(p[2], CLOSE_CM):
                    claim = True
                    break
                tm = self.T[m]
                if tm.sure is not None and t - tm.sure[0] < 2.0 and \
                        math.hypot(sp["mx"] - tm.sure[1], sp["my"] - tm.sure[2]) <= CLOSE_CM:
                    claim = True
                    break
            if not claim and self.pair_guard:
                # (e18) a spot on another robot's OWN pose is that robot's as likely as anyone's: never eliminated onto
                # (5 Nov 14:55: Left's own pose was 0.1 cm from the spot given to Right)
                sp["own_of"] = None
                for m in self.names:
                    o = self.T[m].own[-1] if self.T[m].own else None
                    if o is not None and math.hypot(sp["mx"] - o[1], sp["my"] - o[2]) <= OWN_ON_SPOT_CM:
                        sp["own_of"] = m
                        break
            if not claim:
                cands.append(sp)
        if not cands:
            return out
        t0 = max(sp["pts"][0][0] for sp in cands) if cands else t
        standing = [n for n in unacc if self._standing(self.T[n], max(t - STEADY_S, min(t0, t - STEADY_S)), t)]
        if not standing:
            return out
        if len(standing) == 1 and len(cands) == 1 and len(unacc) == 1 and self.pair_guard and (
                n_unacc > 1 or (cands[0].get("own_of") not in (None, standing[0]))):
            # (e18) another unaccounted robot (in a pass) or another robot's own pose on the spot: not guessed
            self.counts["elim_refused_guard"] = self.counts.get("elim_refused_guard", 0) + 1
            if t >= self.elim_wait.get(standing[0] + ":guard", -1e9):
                self.elim_wait[standing[0] + ":guard"] = t + 5.0
                self.log("tracking", dict(t=round(t, 2), robot=standing[0], what="elimination refused",
                                          why="%s" % ("%d robots unaccounted (a close pass included)" % n_unacc
                                                      if n_unacc > 1 else "the spot is on %s's own pose" %
                                                      cands[0].get("own_of"))))
            return out
        need_proof = False
        if len(standing) == 1 and len(cands) == 1 and len(unacc) == 1:
            n, sp = standing[0], cands[0]
            if sp.get("last") is None or sp["t"] < t:
                return out                  # not in this picture: next one
            d = sp["last"]
            tr = self.T[n]
            p = preds.get(n)
            dist = math.hypot(sp["mx"] - p[0], sp["my"] - p[1]) if p is not None else None
            # (e19, audit H4) further from its own pose than its stated doubt (and the camera's noise) allows: never
            # named by elimination alone; the wiggle / out-and-back below must show it is that sighting
            o_ = tr.own[-1] if tr.own else None
            d_own = math.hypot(sp["mx"] - o_[1], sp["my"] - o_[2]) if o_ is not None else dist
            fn = getattr(self, "proof_fn", None)
            if fn is not None and d_own is not None:
                try:
                    need_proof = d_own > float(fn(n, max(OB_SD_FLOOR, self._sd(sp["pts"]), d.sigma or 0.0)))
                except Exception:  # noqa: BLE001
                    need_proof = False
            if need_proof:
                self.counts["elim_needs_proof"] = self.counts.get("elim_needs_proof", 0) + 1
                if t >= self.elim_wait.get(n + ":proof_said", -1e9):
                    self.elim_wait[n + ":proof_said"] = t + 10.0
                    self.log("tracking", dict(t=round(t, 2), robot=n, what="elimination needs motion proof",
                                              why="the only steady sighting is %.0f cm from its own pose, beyond its "
                                              "stated doubt: a wiggle / out-and-back first" % d_own))
        if len(standing) == 1 and len(cands) == 1 and len(unacc) == 1 and not need_proof:
            fix = Det(d.src, t, sp["mx"], sp["my"], d.th, None, d.sigma / math.sqrt(max(1, len(sp["pts"]))),
                      d.score, d.area)
            self._take(n, t, sp, fix, dist, "elimination")
            out[n] = (fix, "elimination")
            return out
        # ambiguous: ask one unaccounted robot to wiggle (the longest unaccounted first)
        self.counts["elim_ambiguous"] += 1
        w = min(standing, key=lambda n: (self.T[n].seen_t or -1e9, self.names.index(n)))
        if self.wiggle_fn is None or t < self.elim_wait.get(w, -1e9):
            if self.elim_wait.get(w) is None:
                self.elim_wait[w] = t + WIGGLE_RETRY_S
                self.log("tracking", dict(t=round(t, 2), robot="+".join(standing), what="elimination unsure",
                                          why="%d robots unaccounted, %d steady sightings nobody claims: not guessed%s"
                                          % (len(unacc), len(cands), "" if self.wiggle_fn else " (no wiggle here)")))
            return out
        tries = self.elim_wait.get(w + ":tries", 0) + 1
        self.elim_wait[w + ":tries"] = tries
        cl = [dict(x=sp["mx"], y=sp["my"], sd0=self._sd(sp["pts"]), sig=sp["sig"], n0=len(sp["pts"]), seen=[])
              for sp in cands]
        fails = self.id_fails.setdefault(w, dict(wiggle=0, outback=0))
        kinds = ["wiggle", "outback"] if (not self.outback_on or fails["wiggle"] <= fails["outback"]) else \
            ["outback", "wiggle"]
        if not self.outback_on:
            kinds = ["wiggle"]
        for kind in kinds:
            req = dict(kind=kind, spots=[(c["x"], c["y"]) for c in cl])
            if kind == "outback":
                req.update(self._outback_size(cl))
                if req.get("too_noisy"):
                    # (e19) the camera's noise needs a drive longer than any safe out-and-back: nothing guessed
                    fails[kind] += 1
                    self.counts["outback_too_noisy"] = self.counts.get("outback_too_noisy", 0) + 1
                    self.log("tracking", dict(t=round(t, 2), robot=w, what="out-and-back refused",
                                              why="the camera's noise (%.2f cm) needs %.1f cm, over %.0f cm" % (
                                                  req["sig"], req["L"], OB_MAX_SOUND_CM)))
                    continue
            self.elim_wig = dict(robot=w, t0=t, tries=tries, unacc=list(unacc), cands=cl, kind=kind, req=req)
            self.log("tracking", dict(t=round(t, 2), robot=w, what="%s asked" % kind,
                                      why="elimination: %d robots unaccounted (%s), %d steady sightings nobody claims%s"
                                      % (len(unacc), ", ".join(unacc), len(cands), "" if kind == "wiggle" else
                                         "; out and back %.1f cm at %.1f cm/s (its camera noise %.2f cm)" % (
                                             req["L"], req["v"], req["sig"]))))
            self.wiggle_spots = req["spots"]
            self.wiggle_req = req
            try:
                ok = self.wiggle_fn(w, "which of %d steady camera sightings it is" % len(cands))
            finally:
                self.wiggle_spots = None
                self.wiggle_req = None
            if ok is False:
                # (e18) the run refused it (no turn / drive is safe at every candidate): the other kind, at once
                fails[kind] += 1
                self.counts["%s_refused" % kind] += 1
                self.elim_wig = None
                continue
            self.counts["elim_wiggles" if kind == "wiggle" else "outbacks"] += 1
            return out
        self.elim_wait[w] = t + min(ELIM_RETRY_MAX_S, WIGGLE_RETRY_S * tries)
        self.log("tracking", dict(t=round(t, 2), robot=w, what="identity move refused",
                                  why="no wiggle%s is safe at every candidate: not guessed; asked again in %.0f s" % (
                                      " or out-and-back" if self.outback_on else "", self.elim_wait[w] - t)))
        return out

    def _outback_size(self, cl):
        """(e18) The out-and-back's length and speed from the candidates' MEASURED camera noise (their standing
        scatter, the largest of them: the drive must show on whichever one it is) and the layer's measured picture gap:
        L = OB_K x sigma x sqrt(2 / OB_N) within Nick's 2-4 cm; v = L / (OB_N x picture gap), so OB_N pictures see it at
        the far end, within 2-5 cm/s."""
        sig = max([max(OB_SD_FLOOR, c["sd0"]) for c in cl] or [OB_SD_FLOOR])
        need = OB_K * sig * math.sqrt(2.0 / OB_N)          # what the far end's mean must show past its noise
        if OB_SOUND:
            # (e19, audit H3: capped at 4 cm, above ~0.9 cm of noise the "moved" bar fell under the 4 sigmas it is
            # described as) as long as the noise needs, up to OB_MAX_SOUND_CM; the controller refuses it where there
            # is no room for that length (never shortened below it: a shorter drive could not be told from noise)
            L = max(OB_MIN_CM, need / OB_FAR_SHARE)
        else:
            L = min(OB_MAX_CM, max(OB_MIN_CM, need / OB_FAR_SHARE))
        ts = self.frame_ts[-21:]
        gaps = sorted(b - a for a, b in zip(ts, ts[1:]) if b > a)
        gap = gaps[len(gaps) // 2] if gaps else 0.25
        v = min(OB_V_MAX, max(OB_V_MIN, L / (2.0 * OB_N * gap)))     # OB_N pictures on the outer half of the way out
        thr = max(0.5 * L, min(OB_FAR_SHARE * L, need))
        return dict(L=round(L, 2), v=round(v, 2), sig=round(sig, 3), thr=round(thr, 2), gap=round(gap, 3),
                    win=round(2.0 * L / v + OB_SLACK_S, 2), L_need=round(L, 2) if OB_SOUND else None,
                    too_noisy=bool(OB_SOUND and L > OB_MAX_SOUND_CM))

    def _take(self, n, t, sp, fix, dist, how):
        tr = self.T[n]
        o = self._own_at(tr, t)
        trav = self._travel(tr, tr.sure[0], t) if tr.sure is not None else None
        if dist is not None:
            self.record(n, t, dist, trav)
        tr.elim = dict(t=t, x=fix.x, y=fix.y, own0=(o[1], o[2]) if o else None)
        tr.unsure = None
        self.flags.pop(n, None)
        if sp in self.spots:
            self.spots.remove(sp)
        self.counts["eliminated" if how == "elimination" else "elim_wiggle_named"] += 1
        self.log("tracking", dict(t=round(t, 2), robot=n, what="named by %s" % how, d_cm=round(dist, 1) if dist
                                  is not None else "", x=round(fix.x, 1), y=round(fix.y, 1),
                                  why="a steady camera sighting nobody else can claim, %s standing and unaccounted"
                                  % n if how == "elimination" else "the sighting that went out and back with its drive"
                                  if "out-and-back" in how else "the sighting that turned during its wiggle"))

    def wiggle_started_elim(self, name, t):
        if self.elim_wig is not None and self.elim_wig["robot"] == name:
            self.elim_wig["t0"] = t

    def _elim_wiggle_watch(self, t):
        ew = self.elim_wig
        if ew.get("kind") == "outback":
            return self._outback_watch(t)
        if t - ew["t0"] < 0.2:
            return None
        for sp in self.spots:
            if sp["t"] != t:
                continue
            near = sorted((math.hypot(sp["mx"] - c["x"], sp["my"] - c["y"]), k) for k, c in enumerate(ew["cands"]))
            if near and near[0][0] < WIGGLE_NEAR_CM:
                q = sp["pts"][-1]
                ew["cands"][near[0][1]]["seen"].append(q)
        if t - ew["t0"] < 1.6 * WIGGLE_S:
            return None
        self.elim_wig = None
        hits = []
        for k, c in enumerate(ew["cands"]):
            pts = c["seen"]
            if not pts:
                continue
            spin = any(q[4] for q in pts)
            ths = [q[5] for q in pts if q[5] is not None]
            th_turn = len(ths) >= 2 and max(abs(((a - ths[0] + 180) % 360) - 180) for a in ths) >= WIG_TH_DEG
            sd = self._sd(pts)
            scat = sd >= max(WIG_SD_ABS, WIG_SD_K * c["sd0"])
            mx = sum(q[1] for q in pts) / len(pts)
            my = sum(q[2] for q in pts) / len(pts)
            stayed = math.hypot(mx - c["x"], my - c["y"]) <= max(WIGGLE_NEAR_CM / 2.0, 3 * c["sig"])
            if (spin or th_turn or scat) and stayed:
                # (e18, found by its simulator case) the fix is where it is NOW: its last sightings, not the mean over
                # the whole watch (a robot that went on after its wiggle was set back to where it had wiggled)
                last = pts[-OB_N:]
                mx = sum(q[1] for q in last) / len(last)
                my = sum(q[2] for q in last) / len(last)
                hits.append((k, mx, my, "spin" if spin else ("heading" if th_turn else "scatter %.1f cm" % sd)))
        w = ew["robot"]
        if len(hits) == 1:
            k, mx, my, why = hits[0]
            c = ew["cands"][k]
            sp = next((s_ for s_ in self.spots if math.hypot(s_["mx"] - c["x"], s_["my"] - c["y"]) < WIGGLE_NEAR_CM),
                      None)
            src = sp["last"].src if sp is not None and sp.get("last") is not None else "motion"
            fix = Det(src, t, mx, my, None, None, SOURCE_SIGMA.get(src, 1.0))
            p = self.predict(w, t)
            self._take(w, t, sp, fix, math.hypot(mx - p[0], my - p[1]) if p else None, "wiggle (%s)" % why)
            self.elim_wait.pop(w + ":tries", None)
            return w, fix
        self.counts["elim_wiggle_none"] += 1
        self.id_fails.setdefault(w, dict(wiggle=0, outback=0))["wiggle"] += 1
        self.elim_wait[w] = t + min(ELIM_RETRY_MAX_S, WIGGLE_RETRY_S * ew["tries"])
        if self.outback_on:
            self.elim_wait[w] = t + 0.5          # (e18) the out-and-back next, at once (the wiggle could not tell)
        self.log("tracking", dict(t=round(t, 2), robot=w, what="wiggle showed nothing" if not hits else
                                  "wiggle unclear: not guessed", why="%d of %d steady sightings turned; asked again "
                                  "in %.0f s" % (len(hits), len(ew["cands"]), self.elim_wait[w] - t)))
        return None

    def _outback_watch(self, t):
        """(e18) While one robot drives out and back: every candidate's sightings this picture. The candidate whose
        far-end mean (its OB_N sightings furthest from where it stood) is past the threshold while every other one's is
        under half of it, is that robot: named as by a wiggle (a correction). Decided once the moved one is back (or at
        the window's end); none or several moved: nothing is guessed, asked again later."""
        ew = self.elim_wig
        req = ew["req"]
        for sp in self.spots:
            if sp["t"] != t:
                continue
            near = sorted((math.hypot(sp["mx"] - c["x"], sp["my"] - c["y"]), k) for k, c in enumerate(ew["cands"]))
            q = sp["pts"][-1]
            near2 = sorted((math.hypot(q[1] - c["x"], q[2] - c["y"]), k) for k, c in enumerate(ew["cands"]))
            if near2 and near2[0][0] < max(WIGGLE_NEAR_CM, 3 * req["L"]):
                ew["cands"][near2[0][1]]["seen"].append(q)
        if OB_SOUND:
            # (e19, audit H3: "the bar is applied to the 3 farthest sightings of a whole watch, which grows with the
            # number of pictures") WHEN it was out comes from the driving robot's OWN track (its wheels); each
            # candidate's MEDIAN distance from where it stood over the pictures around that time is its "far"; the bar
            # is half the drive and OB_K sigmas over those pictures' count. Its own track not moving = the drive did
            # not happen: nothing is named (a lost command). Decided at the window's end.
            if t - ew["t0"] < req["win"]:
                return None
            fars, thr, own_moved = self._outback_medians(ew, t)
            back = True
        else:
            def far(c):
                d = sorted((math.hypot(q[1] - c["x"], q[2] - c["y"]) for q in c["seen"]), reverse=True)[:OB_N]
                return sum(d) / len(d) if len(d) >= min(OB_N, 2) else 0.0
            fars = [far(c) for c in ew["cands"]]
            thr = req["thr"]
            own_moved = True
        moved = [k for k, f in enumerate(fars) if f >= thr] if own_moved else []
        if not OB_SOUND:
            back = False
            if len(moved) == 1:
                c = ew["cands"][moved[0]]
                last = c["seen"][-OB_N:]
                if len(last) >= 2:
                    mx = sum(q[1] for q in last) / len(last)
                    my = sum(q[2] for q in last) / len(last)
                    back = math.hypot(mx - c["x"], my - c["y"]) <= 0.5 * thr
            if not back and t - ew["t0"] < req["win"]:
                return None
        self.elim_wig = None
        w = ew["robot"]
        # every other candidate stayed: under the threshold and clearly less than the one that moved (half the drive;
        # e18 batch: 2.3 against a standing one's 1.0 cm of noise maxima was refused by the first "half the threshold")
        quiet = all(f < min(thr, fars[moved[0]] - 0.5 * req["L"]) for k, f in enumerate(fars) if k not in moved) \
            if len(moved) == 1 else False
        if len(moved) == 1 and quiet:
            c = ew["cands"][moved[0]]
            sp = min(self.spots, key=lambda s_: math.hypot(s_["mx"] - c["x"], s_["my"] - c["y"])) if self.spots else None
            if sp is not None and math.hypot(sp["mx"] - c["x"], sp["my"] - c["y"]) > WIGGLE_NEAR_CM:
                sp = None
            src = sp["last"].src if sp is not None and sp.get("last") is not None else "motion"
            last = c["seen"][-OB_N:]
            mx = sum(q[1] for q in last) / len(last)
            my = sum(q[2] for q in last) / len(last)
            fix = Det(src, t, mx, my, None, None, SOURCE_SIGMA.get(src, 1.0))
            p = self.predict(w, t)
            self._take(w, t, sp, fix, math.hypot(mx - p[0], my - p[1]) if p else None,
                       "wiggle (out-and-back %.1f cm)" % fars[moved[0]])
            self.counts["outback_named"] += 1
            self.identity_evidence(w, True, t, "an out-and-back named it")       # (e20)
            self.elim_wait.pop(w + ":tries", None)
            self.id_fails.pop(w, None)
            return w, fix
        self.counts["outback_none"] += 1
        self.id_fails.setdefault(w, dict(wiggle=0, outback=0))["outback"] += 1
        self.elim_wait[w] = t + min(ELIM_RETRY_MAX_S, WIGGLE_RETRY_S * ew["tries"])
        self.log("tracking", dict(t=round(t, 2), robot=w, what="out-and-back unclear: not guessed",
                                  why="%d of %d candidates moved past %.1f cm (far ends %s); asked again in %.0f s" % (
                                      len(moved), len(fars), thr, ", ".join("%.1f" % f for f in fars),
                                      self.elim_wait[w] - t)))
        return None

    def _outback_medians(self, ew, t):
        """(e19) ([median far distance per candidate], the bar, whether the driving robot's own track moved)."""
        req = ew["req"]
        w = ew["robot"]
        own = [q for q in self.T[w].own if q[0] >= ew["t0"] - 0.3] if self.T[w].own else []
        L, v = float(req["L"]), max(0.5, float(req["v"]))
        if own:
            o0 = own[0]
            dists = [(math.hypot(q[1] - o0[1], q[2] - o0[2]), q[0]) for q in own]
            dmax, t_far = max(dists)
        else:
            dmax, t_far = 0.0, ew["t0"] + L / v
        own_moved = dmax >= 0.5 * L
        half = max(0.4, 0.5 * L / v)
        fars, ns = [], []
        for c in ew["cands"]:
            d = sorted(math.hypot(q[1] - c["x"], q[2] - c["y"]) for q in c["seen"] if abs(q[0] - t_far) <= half)
            fars.append(d[len(d) // 2] if d else 0.0)
            ns.append(len(d))
        n = max(1, min([k for k in ns if k] or [1]))
        thr = max(0.5 * L, OB_K * float(req["sig"]) / math.sqrt(n))
        return fars, thr, own_moved

    def _probation(self, t):
        """An elimination is confirmed once the robot's own wheels drive ELIM_VERIFY_CM and the camera's sightings
        named for it follow; if the spot stays where it was while the robot drives, it was not the robot: revoked (the
        spot is remembered as not a robot, the run told)."""
        for n, tr in self.T.items():
            e = tr.elim
            if e is None or e.get("own0") is None:
                continue
            o = self._own_at(tr, t)
            if o is None:
                continue
            od = math.hypot(o[1] - e["own0"][0], o[2] - e["own0"][1])
            if od < ELIM_VERIFY_CM:
                continue
            cams = [c for c in tr.cam if c[0] > e["t"] + 0.05]
            moved = math.hypot(cams[-1][1] - e["x"], cams[-1][2] - e["y"]) if cams else 0.0
            if cams and moved >= ELIM_FOLLOW * od:
                tr.elim = None
                self.counts["elim_confirmed"] += 1
                self.identity_evidence(n, True, t, "its own drive confirmed the elimination")   # (e20)
                self.log("tracking", dict(t=round(t, 2), robot=n, what="elimination confirmed",
                                          why="the camera followed its own drive of %.1f cm" % od))
                continue
            if od < 2.0 * ELIM_VERIFY_CM and tr.seen_t is not None and t - tr.seen_t <= self.unacc_after(n):
                continue                     # not far enough yet to tell (a still spot named each picture is no proof)
            tr.elim = None
            tr.sure = None
            self.not_robot.append(dict(x=e["x"], y=e["y"], t=t))
            self.revoked.append(dict(robot=n, t=t, x=e["x"], y=e["y"]))
            self.counts["elim_revoked"] += 1
            self.identity_evidence(n, False, t, "its own drive revoked the elimination")    # (e20)
            self.log("tracking", dict(t=round(t, 2), robot=n, what="elimination revoked",
                                      why="the spot at %.1f, %.1f moved %.1f cm while its own wheels drove %.1f cm: "
                                          "not it" % (e["x"], e["y"], moved, od)))

    def take_revoked(self):
        o = self.revoked
        self.revoked = []
        return o

    # -- (e17, 4) the ML detector on demand ---------------------------------------------------------------------
    def want_detector(self, name, why=None):
        """The run asks for the detector on `name` (a heading check, its own pose disagreeing); None: no longer."""
        if why:
            self.want_ml[name] = str(why)
        else:
            self.want_ml.pop(name, None)

    def _ml_on_demand(self, t, preds):
        """Runs the detector (ml_fn) only for robots that need it: unaccounted, unsure (close pass, wiggle), asked
        (want_detector: heading check, disagreement), at most ML_ROBOT_HZ each and ML_TOTAL_HZ in all. Its sightings go
        in as unnamed "ml" sightings (the same matching and identity rules)."""
        self.ml_calls = [c for c in self.ml_calls if t - c < 1.0]
        out = []
        for n in self.names:
            tr = self.T[n]
            why = self.want_ml.get(n) or (tr.unsure and "unsure: %s" % tr.unsure) or \
                (self._unaccounted(n, t) and "unaccounted") or \
                (n in self.__dict__.get("focus", {}) and "searched for (its error shape)")
            if not why:
                continue
            if t - self.ml_last.get(n, -1e9) < 1.0 / ML_ROBOT_HZ or len(self.ml_calls) >= ML_TOTAL_HZ:
                continue
            p = preds.get(n)
            self.ml_last[n] = t
            self.ml_calls.append(t)
            self.counts["ml_calls"] += 1
            try:
                got = self.ml_fn(n, t, why, None if p is None else (p[0], p[1], max(p[2], self.gate_max(n)))) or []
            except Exception:  # noqa: BLE001  (a failing detector never stops the layer)
                got = []
            for d in got:
                d.src, d.name = "ml", None
                if d.sigma is None:
                    d.sigma = SOURCE_SIGMA["ml"]
                out.append(d)
            self.counts["ml_dets"] += len(got)
        return out

    def _clusters(self, dets):
        groups = []
        for d in sorted(dets, key=lambda q: SOURCE_RANK.get(q.src, 5)):
            g = next((g for g in groups if any(math.hypot(d.x - e.x, d.y - e.y) < CLUSTER_CM for e in g)), None)
            if g is None:
                groups.append([d])
            else:
                g.append(d)
        out = []
        for g in groups:
            best = g[0]
            area = max(e.area for e in g)
            if area > best.area:
                best = Det(best.src, best.t, best.x, best.y, best.th, best.name, best.sigma, best.score, area,
                           best.spin or any(e.spin for e in g), best.lock_id, best.cam, best.light)
            elif any(e.spin for e in g) and not best.spin:
                best.spin = True
            if len(g) > 1:
                best.members = list(g)          # (e20) every camera's sighting of this object, for its checks
            out.append(best)
        return out

    def _camera_check(self, n, t, d):
        """(e10, C4) The camera watch, rebuilt after its false alarms (2 of 12 e09 runs). Their cause: each vote compared
        the turn between two 6 cm chords of SINGLE noisy sightings with the robot's own heading change; for the lock-on's
        0.8 cm a chord turn's noise is ~19 deg (sqrt(6) x sigma / 6 cm), as large as the turn it was checking (~17 deg
        on a 20 cm curve), so ~10 % of votes went the wrong way and 4 of 6 now and then did. Now: each checkpoint is the
        AVERAGE of the last CAM_AVG_S of sightings, checkpoints CAM_CHECK2_CM apart, the robot's own heading taken at
        the same moments, and a vote only when the expected chord turn (half the own heading change over the three
        checkpoints, turning one way all along) is over 3 x its noise: agreeing in sign = good; the other way by over
        2 x its noise = bad; else no vote. MIRLT_CAMERA_WATCH_METHOD=e09: the old check."""
        if CAM_WATCH_METHOD == "e09":
            return self._camera_check_e09(n, t, d)
        tr = self.T[n]
        if not tr.own or tr.own[-1][3] is None:
            return self.cam_ok
        cp = self.cam_ref.setdefault(n, [])
        if cp and t - cp[-1][0] > CAM_WIN_S:
            del cp[:]
        recent = [q for q in tr.cam if t - q[0] <= CAM_AVG_S]
        if len(recent) < CAM_AVG_MIN:
            return self.cam_ok
        ax = sum(q[1] for q in recent) / len(recent)
        ay = sum(q[2] for q in recent) / len(recent)
        tc = sum(q[0] for q in recent) / len(recent)
        if cp and math.hypot(ax - cp[-1][1], ay - cp[-1][2]) < CAM_CHECK2_CM:
            return self.cam_ok
        o = self._own_at(tr, tc)
        if o is None or o[3] is None:
            return self.cam_ok
        cp.append((tc, ax, ay, float(o[3]), d.sigma / math.sqrt(len(recent))))
        del cp[:-3]
        if len(cp) < 3 or cp[2][0] - cp[0][0] > CAM_WIN_S:
            return self.cam_ok
        (_, x0, y0, h0, s0), (_, x1, y1, h1, s1), (_, x2, y2, h2, s2) = cp
        t01 = ((h1 - h0 + 180.0) % 360.0) - 180.0
        t12 = ((h2 - h1 + 180.0) % 360.0) - 180.0
        if t01 * t12 <= 0:
            return self.cam_ok               # its own heading did not turn one way all along (an S-bend, a stop): no vote
        expect = (t01 + t12) / 2.0           # a chord turns half as much as the heading over its two chords
        L = 0.5 * (math.hypot(x1 - x0, y1 - y0) + math.hypot(x2 - x1, y2 - y1))
        sd = math.degrees(math.sqrt(s0 * s0 + 4.0 * s1 * s1 + s2 * s2) / max(1e-6, L))
        if abs(expect) < max(CAM_MIN_TURN_DEG, 3.0 * sd):
            return self.cam_ok
        a1, a2 = math.atan2(y1 - y0, x1 - x0), math.atan2(y2 - y1, x2 - x1)
        cam_turn = math.degrees(math.atan2(math.sin(a2 - a1), math.cos(a2 - a1)))
        if cam_turn * expect > 0:
            self._cam_vote(t, True, n)
        elif abs(cam_turn) >= 2.0 * sd:
            self._cam_vote(t, False, n)
        return self.cam_ok

    def _camera_check_e09(self, n, t, d):
        """Observe, then trust (the camera itself). The camera's track of a robot must TURN the way the robot's own gyro
        says it turned (a frame-free check: a wrong camera name, a mirrored or swapped calibration turns the other way;
        positions cannot be compared, they are pulled to the camera). Every CAM_CHECK_CM of camera travel a checkpoint;
        over three checkpoints the camera's turn against its own heading's change, when that is over CAM_TURN_DEG.
        CAM_BAD_N of the last CAM_CHECK_N votes (any robots) the other way: the picture is not trusted (no camera
        fixes; the robots drive on their filters and the run narrows); trusted again after CAM_GOOD_N agreeing votes in a
        row (the votes go on while it is not trusted). Returns whether this fix may be used."""
        tr = self.T[n]
        o = tr.own[-1] if tr.own else None
        if o is None or o[3] is None:
            return self.cam_ok
        cp = self.cam_ref.setdefault(n, [])
        if cp and t - cp[-1][0] > 10.0:
            del cp[:]
        if not cp or math.hypot(d.x - cp[-1][1], d.y - cp[-1][2]) >= CAM_CHECK_CM:
            cp.append((t, d.x, d.y, float(o[3])))
            del cp[:-3]
            if len(cp) == 3:
                (_, x0, y0, h0), (_, x1, y1, _), (_, x2, y2, h2) = cp
                a1, a2 = math.atan2(y1 - y0, x1 - x0), math.atan2(y2 - y1, x2 - x1)
                cam_turn = math.degrees(math.atan2(math.sin(a2 - a1), math.cos(a2 - a1)))
                own_turn = ((h2 - h0 + 180.0) % 360.0) - 180.0
                if abs(own_turn) >= CAM_TURN_DEG and abs(cam_turn) >= 0.3 * CAM_TURN_DEG:
                    self._cam_vote(t, (cam_turn > 0) == (own_turn > 0), n)
                elif abs(own_turn) >= CAM_TURN_DEG and abs(cam_turn) < 0.15 * CAM_TURN_DEG:
                    pass                     # a straight camera track while it turned a lot: no vote (a turn on its spot)
        return self.cam_ok

    @property
    def proven(self):
        return (not self.camera_watch) or (self.cam_ok and self.cam_good >= CAM_GOOD_N)

    def _cam_vote(self, t, ok, robot="-"):
        if ok:
            self.cam_good += 1
        self.cam_checks.append(ok)
        del self.cam_checks[:-CAM_CHECK_N]
        self.cam_bad_by = (getattr(self, "cam_bad_by", []) + [robot if not ok else None])[-CAM_CHECK_N:]
        self.counts["camera_votes"] = self.counts.get("camera_votes", 0) + 1
        if not ok:
            self.counts["camera_votes_bad"] = self.counts.get("camera_votes_bad", 0) + 1
        bad_robots = {r for r in self.cam_bad_by if r is not None}
        if self.cam_ok and self.cam_checks.count(False) >= CAM_BAD_N and \
                len(bad_robots) >= min(2, len(self.names)):
            self.cam_ok = False
            self.counts["camera_distrusted"] = self.counts.get("camera_distrusted", 0) + 1
            self.log("tracking", dict(t=round(t, 2), robot="-", what="camera not trusted",
                                      why="%d of the last %d checks: the picture turned the robots the other way from "
                                          "their own gyros" % (self.cam_checks.count(False), len(self.cam_checks))))
        elif not self.cam_ok and len(self.cam_checks) >= CAM_GOOD_N and all(self.cam_checks[-CAM_GOOD_N:]):
            self.cam_ok = True
            self.log("tracking", dict(t=round(t, 2), robot="-", what="camera trusted again", why="checks agree"))

    def _assign(self, t, want, preds, free):
        """Least total distance inside the gates; contested / merged sightings name nobody."""
        cand = {}
        for n in want:
            px, py, g = preds[n]
            cand[n] = [(math.hypot(d.x - px, d.y - py), d) for d in free if math.hypot(d.x - px, d.y - py) <= g]
        res = {}
        for d in free:
            near = sorted((math.hypot(d.x - preds[n][0], d.y - preds[n][1]), n) for n in want
                          if math.hypot(d.x - preds[n][0], d.y - preds[n][1]) <= preds[n][2])
            if len(near) >= 2 and d.area >= MERGE_AREA:
                self.counts["merged"] += 1
                for _, n in near[:2]:
                    self._mark(n, t, "merged outline")
                for _, n in near:
                    cand[n] = [c for c in cand[n] if c[1] is not d]
            elif len(near) >= 2 and near[1][0] - near[0][0] < CONTESTED_CM and self.identity != "predict_loose":
                self.counts["contested"] += 1
                for _, n in near[:2]:
                    cand[n] = [c for c in cand[n] if c[1] is not d]
        ns = [n for n in want if cand[n]]
        if not ns:
            return res
        best, best_cost = None, None
        # small numbers of robots: every assignment (3 robots x a few sightings)
        opts = [[None] + [c[1] for c in sorted(cand[n], key=lambda q: q[0])[:4]] for n in ns]
        for combo in itertools.product(*opts):
            used = [id(d) for d in combo if d is not None]
            if len(used) != len(set(used)):
                continue
            cost = 0.0
            for n, d in zip(ns, combo):
                cost += (preds[n][2] * 1.2) if d is None else math.hypot(d.x - preds[n][0], d.y - preds[n][1])
            # prefer better sources for the same robot at equal distance
            cost += 0.01 * sum(SOURCE_RANK.get(d.src, 5) for d in combo if d is not None)
            if best_cost is None or cost < best_cost:
                best, best_cost = combo, cost
        for n, d in zip(ns, best):
            if d is None:
                continue
            # (e10) a sighting several times nearer ANOTHER robot's prediction, inside that robot's gate, is that robot's
            # second sighting (its lights beside its outline), not this one's: a robot whose gate grew out of view
            # (a probe into unseen ground) took a neighbour's lights and lost itself (rehearsal b)
            dn = math.hypot(d.x - preds[n][0], d.y - preds[n][1])
            if self.cluster and any(m != n and preds.get(m) is not None and math.hypot(d.x - preds[m][0], d.y - preds[m][1]) <=
                   min(preds[m][2], DUPLICATE_RATIO * dn) for m in self.names):
                self.counts["duplicates"] = self.counts.get("duplicates", 0) + 1
                continue
            res[n] = d
        # several sources may see the same robot: keep the best-ranked one per robot (already one per robot here)
        return res

    def _mark(self, n, t, why):
        self.log("tracking", dict(t=round(t, 2), robot=n, what="unsure", why=why))

    # -- close passes and wiggles ------------------------------------------------------------------------------------
    def _in_pair(self, n):
        return any(n in k and v["state"] in ("close", "wiggle") for k, v in self.pairs.items())

    def _pairs(self, t, preds):
        if self.identity == "predict":
            return
        for a, b in itertools.combinations(self.names, 2):
            pa, pb = preds.get(a), preds.get(b)
            if pa is None or pb is None:
                continue
            k = frozenset((a, b))
            d = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
            st = self.pairs.get(k)
            if d < CLOSE_CM and (st is None or st["state"] == "done") and self.eliminate and (
                    self._unaccounted(a, t) or self._unaccounted(b, t)):
                continue        # (e17) an unaccounted robot's prediction is its drifted own pose, not where it is: no
                                # close pass on it (the elimination's claim rule keeps its sightings off the other)
            if d < CLOSE_CM and (st is None or st["state"] == "done"):
                self.pairs[k] = dict(t0=t, state="close", parted_t=None, a=a, b=b,
                                     own0={a: self._own_at(self.T[a], t), b: self._own_at(self.T[b], t)},
                                     cam0={a: self.T[a].sure, b: self.T[b].sure})
                self.counts["close_passes"] += 1
                self.log("tracking", dict(t=round(t, 2), robot="%s+%s" % (a, b), what="close pass",
                                          d_cm=round(d, 1)))
            elif st is not None and st["state"] == "close" and d > PART_CM and st["parted_t"] is None:
                st["parted_t"] = t
            elif st is not None and st["state"] == "close" and st["parted_t"] is None and PAIR_STAND_PART and \
                    d >= CLOSE_CM and self._both_still(a, b, t):
                # (e20, the hard simulator's measured seed 1: Center and Right passed at 15.5 cm, then both stood 17 cm
                # apart, lost, for 105 s: never past PART_CM, so neither took a fix and no wiggle was asked) two robots
                # that both stand still beyond the close distance cannot be flapping in and out of it: parted now, so
                # who is who is checked (kept / swapped / a wiggle) instead of waiting for a move that never comes
                st["parted_t"] = t
                self.counts["pair_parted_standing"] = self.counts.get("pair_parted_standing", 0) + 1

    def _both_still(self, a, b, t):
        """(e20) Both robots' own wheels have stood still for PARKED_STILL_S (their own records, still flag)."""
        for n in (a, b):
            h = [q for q in self.T[n].own if q[0] >= t - PARKED_STILL_S]
            if len(h) < 2 or h[0][0] > t - PARKED_STILL_S + 0.5 or not all(q[4] for q in h):
                return False
        return True

    def _parted(self, t):
        for k, st in list(self.pairs.items()):
            if st["state"] != "close" or st["parted_t"] is None:
                continue
            if t - st["parted_t"] < 0.6:
                continue                # a few sightings each after they parted
            a, b = st["a"], st["b"]
            verdict, ev = self._swap_check(t, st)
            if verdict == "kept":
                st["state"] = "done"
                self.counts["kept"] += 1
                for n in (a, b):
                    self.T[n].unsure = None
            elif verdict == "swapped":
                st["state"] = "done"
                self.counts["swapped"] += 1
                self._swap(a, b, t, "their motions since the close pass (%s)" % ev)
            else:
                # neither explains it: wiggle first (both take no fix until it is known)
                for n in (a, b):
                    self.T[n].unsure = "identity after a close pass"
                if self.wiggle_fn is None:
                    # nobody can wiggle (the school's runs, a replay): no fixes for either until a tag read names it or
                    # their motion tells them apart (checked again each picture), at most NO_WIGGLE_WAIT_S; then
                    # believed as predicted, said
                    if t - st["parted_t"] < NO_WIGGLE_WAIT_S:
                        continue
                    st["state"] = "done"
                    self.counts["wiggle_none"] += 1
                    for n in (a, b):
                        self.T[n].unsure = None
                    self.log("tracking", dict(t=round(t, 2), robot="%s+%s" % (a, b), what="names kept as predicted",
                                              why="no wiggle possible and their motion did not tell them apart"))
                    continue
                st["state"] = "wiggle"
                w = min((a, b), key=lambda n: self.names.index(n))
                self._ask_wiggle(w, st, t)

    def _ask_wiggle(self, w, st, t):
        tr = self.T[w]
        self.wiggles[w] = dict(t0=t, pair=st, at=self.predict(w, t), other=st["b"] if w == st["a"] else st["a"],
                               seen=[], tries=st.get("tries", 0) + 1)
        st["tries"] = self.wiggles[w]["tries"]
        self.counts["wiggles"] += 1
        self.log("tracking", dict(t=round(t, 2), robot=w, what="wiggle asked",
                                  why="who is who after the close pass with %s is not clear from their motion"
                                  % self.wiggles[w]["other"]))
        self.wiggle_fn(w, "identity after a close pass")
        tr.unsure = "identity wiggle"

    def wiggle_started(self, name, t):
        self.wiggle_started_elim(name, t)
        w = self.wiggles.get(name)
        if w is not None:
            w["t0"] = t
            w["at"] = self.predict(name, t)

    def _swap_check(self, t, st):
        a, b = st["a"], st["b"]
        t0 = max(st["t0"], t - SWAP_WIN_S - 1.0)
        res = {}
        for n in (a, b):
            tr = self.T[n]
            cam = [c for c in tr.cam if c[0] >= st["parted_t"] - 0.3]
            o0, o1 = self._own_at(tr, t0), self._own_at(tr, t)
            c0 = st["cam0"].get(n)
            if not cam or o0 is None or o1 is None or c0 is None:
                return "unknown", "no motion to compare"
            res[n] = dict(cam=(cam[-1][1] - c0[1], cam[-1][2] - c0[2]), own=(o1[1] - st["own0"][n][1],
                                                                             o1[2] - st["own0"][n][2]))
        ca, cb = res[a]["cam"], res[b]["cam"]
        oa, ob = res[a]["own"], res[b]["own"]
        keep = math.dist(ca, oa) + math.dist(cb, ob)
        # swapped: the track called a is b (it started where a was: its displacement from a's start spot equals b's own
        # displacement plus where b started relative to a)
        sa, sb = st["cam0"][a], st["cam0"][b]
        ca_abs = (sa[1] + ca[0], sa[2] + ca[1])
        cb_abs = (sb[1] + cb[0], sb[2] + cb[1])
        swap = math.dist((ca_abs[0] - sb[1], ca_abs[1] - sb[2]), ob) + math.dist((cb_abs[0] - sa[1], cb_abs[1] - sa[2]),
                                                                                  oa)
        moved = max(math.hypot(*oa), math.hypot(*ob))
        ev = "keep %.1f cm, swap %.1f cm" % (keep, swap)
        if moved < MIN_MOTION_CM:
            return "unknown", ev + ", too little motion"
        if keep + SWAP_MARGIN_CM < swap:
            return "kept", ev
        if swap + SWAP_MARGIN_CM < keep:
            return "swapped", ev
        return "unknown", ev

    def _swap(self, a, b, t, why):
        for n in (a, b):
            self.identity_evidence(n, False, t, "a swap found")          # (e20)
        A, B = self.T[a], self.T[b]
        A.sure, B.sure = B.sure, A.sure
        A.cam, B.cam = B.cam, A.cam
        A.unsure = B.unsure = None
        A.lock_seeded = B.lock_seeded = None
        for n in (a, b):            # the locks follow the wrong robots: seed them again on their own
            tr = self.T[n]
            if tr.sure is not None:
                self.seed(n, t, tr.sure[1], tr.sure[2], "swap")
        self.log("tracking", dict(t=round(t, 2), robot="%s+%s" % (a, b), what="names swapped back", why=why))

    def _wiggle_watch(self, t, dets):
        for w, ws in list(self.wiggles.items()):
            st = ws["pair"]
            if t - ws["t0"] < 0.2:
                continue
            pw0, po0 = self.predict(w, t), self.predict(ws["other"], t)
            spins = [d for d in dets if d.src in ("motion", "lock", "ml") and d.spin and any(
                p is not None and math.hypot(d.x - p[0], d.y - p[1]) < WIGGLE_NEAR_CM for p in (pw0, po0))]
            ws["seen"] += [(d.x, d.y) for d in spins]
            if t - ws["t0"] < WIGGLE_S:
                continue
            other = ws["other"]
            pw, po = self.predict(w, t), self.predict(other, t)
            del self.wiggles[w]
            if not ws["seen"] or pw is None or po is None:
                self.counts["wiggle_none"] += 1
                if ws["tries"] < 3:
                    st["state"] = "close"
                    st["parted_t"] = t - 0.6 + WIGGLE_RETRY_S     # asked again later
                    self.T[w].unsure = "identity wiggle"
                    continue
                st["state"] = "done"
                for n in (w, other):
                    self.T[n].unsure = None
                self.log("tracking", dict(t=round(t, 2), robot=w, what="wiggle showed nothing",
                                          why="3 tries: names kept as predicted"))
                continue
            mx = sum(p[0] for p in ws["seen"]) / len(ws["seen"])
            my = sum(p[1] for p in ws["seen"]) / len(ws["seen"])
            near_w = math.hypot(mx - pw[0], my - pw[1])
            near_o = math.hypot(mx - po[0], my - po[1])
            st["state"] = "done"
            if not (near_w < WIGGLE_CLEAR * near_o or near_o < WIGGLE_CLEAR * near_w):
                # the turning outline is about as near both: no verdict, names kept as predicted (said)
                self.counts["wiggle_none"] += 1
                for n in (w, other):
                    self.T[n].unsure = None
                self.log("tracking", dict(t=round(t, 2), robot=w, what="wiggle unclear: names kept",
                                          why="the turning outline was %.1f / %.1f cm from the two" % (near_w, near_o)))
                continue
            if near_w <= near_o:
                self.counts["wiggle_kept"] += 1
                for n in (w, other):
                    self.T[n].unsure = None
                self.log("tracking", dict(t=round(t, 2), robot=w, what="wiggle: names kept",
                                          why="the turning outline was where %s was thought to be" % w))
            else:
                self.counts["wiggle_swapped"] += 1
                self._swap(w, other, t, "the wiggle: the turning outline was where %s was thought to be" % other)

    # -- parked robots and new objects -------------------------------------------------------------------------------
    def _parked(self, n, t, d):
        """A parked robot (its wheels standing PARKED_STILL_S) whose camera spot, averaged (the mean of its first
        PARKED_REF_N sightings standing against the mean of its last PARKED_NOW_N: single sightings are 1-2 cm noisy),
        moved PARKED_MOVE_CM to PARKED_MAX_CM: moved while parked."""
        if not self.parked_watch:
            return
        tr = self.T[n]
        o = tr.own[-1] if tr.own else None
        if o is None or not o[4]:
            tr.parked_at = None
            return
        k = len(tr.own) - 1
        while k > 0 and tr.own[k - 1][4]:
            k -= 1
        if t - tr.own[k][0] < PARKED_STILL_S:
            return
        if tr.parked_at is None:
            tr.parked_at = dict(ref=[], now=[])
        pa = tr.parked_at
        if len(pa["ref"]) < PARKED_REF_N:
            pa["ref"].append((d.x, d.y))
            return
        pa["now"].append((d.x, d.y))
        del pa["now"][:-PARKED_NOW_N]
        if len(pa["now"]) < PARKED_NOW_N:
            return
        rx = sum(p[0] for p in pa["ref"]) / len(pa["ref"])
        ry = sum(p[1] for p in pa["ref"]) / len(pa["ref"])
        nx = sum(p[0] for p in pa["now"]) / len(pa["now"])
        ny = sum(p[1] for p in pa["now"]) / len(pa["now"])
        mv = math.hypot(nx - rx, ny - ry)
        if mv > PARKED_MAX_CM or not self.proven or t - getattr(tr, "parked_ev_t", -1e9) < PARKED_EVERY_S or \
                self._close_to_other(n, nx, ny):
            return
        if mv >= PARKED_MOVE_CM:
            tr.parked_ev_t = t
            self.counts["parked_moved"] += 1
            self.log("tracking", dict(t=round(t, 2), robot=n, what="moved while parked", d_cm=round(mv, 1),
                                      x=round(nx, 1), y=round(ny, 1), src=d.src))
            tr.parked_at = dict(ref=list(pa["now"]), now=[])
            self.parked_events = getattr(self, "parked_events", [])
            self.parked_events.append(dict(t=t, robot=n, x=nx, y=ny, d_cm=mv))

    def _bricks(self, t, dets, preds):
        keep = []
        for u in self.unclaimed:
            if t - u["t"] < 1.0:
                keep.append(u)
        self.unclaimed = keep
        for d in dets:
            if any(p is not None and math.hypot(d.x - p[0], d.y - p[1]) < 2 * GATE_CM for p in preds.values()):
                continue
            if self.own_resets_on and any(tr.own and math.hypot(d.x - tr.own[-1][1], d.y - tr.own[-1][2]) <
                                          BRICK_NOT_NEAR_OWN_CM for tr in self.T.values()):
                continue        # (e10) beside where a robot's own wheels put it: that robot, not a new object
            if any(math.hypot(d.x - o["x"], d.y - o["y"]) < BRICK_R_CM for o in self.obstacles):
                continue
            u = next((u for u in self.unclaimed if math.hypot(d.x - u["x"], d.y - u["y"]) < BRICK_STILL_CM), None)
            if u is None:
                self.unclaimed.append(dict(t0=t, t=t, x=d.x, y=d.y, n=1))
                continue
            u["t"], u["n"] = t, u["n"] + 1
            if t - u["t0"] >= BRICK_S and u["n"] >= 5:
                ob = dict(x=round(u["x"], 1), y=round(u["y"], 1), r=BRICK_R_CM, t=round(t, 2), src="camera: new object")
                self.obstacles.append(ob)
                self.counts["bricks"] += 1
                self.unclaimed.remove(u)
                self.log("tracking", dict(t=round(t, 2), robot="-", what="new object", x=ob["x"], y=ob["y"]))
                self.new_obstacles = getattr(self, "new_obstacles", [])
                self.new_obstacles.append(ob)

    def take_obstacles(self):
        o = getattr(self, "new_obstacles", [])
        self.new_obstacles = []
        return o

    def take_parked(self):
        o = getattr(self, "parked_events", [])
        self.parked_events = []
        return o

    def summary(self):
        return dict(version=VERSION, sources=list(self.sources), identity=self.identity, eliminate=self.eliminate,
                    outback=self.outback_on,
                    gate_record=self.gate_record, ml_on_demand=self.ml_on,
                    drift={n: dict(zip(("per_cm", "p90_cm"), [round(v, 3) for v in self.drift(n)]),
                                   gate_max_cm=round(self.gate_max(n), 1), n=len(tr.rec)) for n, tr in self.T.items()},
                    counts=dict(self.counts), by_source=dict(self.by_src),
                    per_robot={n: dict(tr.src_n) for n, tr in self.T.items()},
                    trust_refused=dict(self.trust_refused))
